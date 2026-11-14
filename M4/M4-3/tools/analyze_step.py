#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Analyze M4-3 speed-loop CSV data (single step, target sequence, or disturbance).

Original fixed-reference command (kept):
    python3 tools/analyze_step.py --input data/step_PID.csv --ref 60
2.5 multi-setpoint sequence command:
    python3 tools/analyze_step.py --input data/sequence_PID.csv --sequence

Firmware CSV header: t_s,cnt,delta_cnt,target_rpm,actual_rpm,pwm
Rows beginning with non-numeric text (for example #TARGET acknowledgements) are ignored.
"""
import argparse
import csv
import os
import numpy as np


def load_csv(path):
    """Read the firmware CSV, locating its header even if capture began earlier."""
    with open(path, newline='', encoding='utf-8-sig') as f:
        rows = list(csv.reader(f))
    if not rows:
        raise SystemExit('[错误] 文件为空: %s' % path)

    header_row = None
    header_index = None
    for i, row in enumerate(rows):
        hdr = [h.strip().lower() for h in row]
        if any(x in hdr for x in ('t', 't_s', 'time', 'time_s')) and \
           any(x in hdr for x in ('actual_rpm', 'actual', 'rpm')):
            header_row, header_index = hdr, i
            break
    if header_row is None:
        raise SystemExit('[错误] 找不到 CSV 表头（需含 t_s 和 actual_rpm）；可先发送 CSV 命令输出表头。')

    def col(*names):
        for name in names:
            if name in header_row:
                return header_row.index(name)
        return None

    it = col('t', 't_s', 'time', 'time_s')
    ir = col('target_rpm', 'target', 'ref_rpm', 'ref')
    iy = col('actual_rpm', 'actual', 'rpm')
    ip = col('pwm', 'u', 'ctrl')
    t, ref, y, pwm = [], [], [], []
    for row in rows[header_index + 1:]:
        if not row or len(row) <= max(it, iy):
            continue
        try:
            t.append(float(row[it]))
            y.append(float(row[iy]))
            ref.append(float(row[ir]) if ir is not None and ir < len(row)
                       and row[ir].strip() else np.nan)
            pwm.append(float(row[ip]) if ip is not None and ip < len(row)
                       and row[ip].strip() else np.nan)
        except ValueError:
            # Skip command replies and other non-numeric status text.
            continue

    if len(t) < 3:
        raise SystemExit('[错误] 有效数据点太少: %d' % len(t))
    return (np.asarray(t, dtype=float), np.asarray(ref, dtype=float),
            np.asarray(y, dtype=float), np.asarray(pwm, dtype=float))

def step_metrics(t, y, ref):
    """Legacy metrics for one fixed-reference upward step."""
    ymax = float(np.max(y))
    overshoot = max(0.0, (ymax - ref) / abs(ref) * 100.0) if ref else 0.0
    t10 = t90 = None
    for i in range(len(y)):
        if t10 is None and y[i] >= 0.1 * ref:
            t10 = t[i]
        if y[i] >= 0.9 * ref:
            t90 = t[i]
            break
    rise = (t90 - t10) if t10 is not None and t90 is not None else None
    span = t[-1] - t[0]
    mask = t >= (t[-1] - 0.3 * span)
    yss = y[mask]
    ss_err = float(np.mean(yss) - ref)
    return {'overshoot': overshoot, 'rise': rise, 'ss_err': ss_err,
            'ss_rip': float(np.max(yss) - np.min(yss)),
            'ss_std': float(np.std(yss)), 'ymax': ymax}


def sequence_segments(t, target):
    """Split samples whenever the firmware-reported target changes."""
    valid = np.isfinite(target)
    indices = np.flatnonzero(valid)
    if len(indices) == 0:
        raise SystemExit('[错误] CSV 没有有效 target_rpm 列；目标序列分析需要该列。')
    # Keep only rows with a valid target, then split on changes larger than 0.01 rpm.
    t, target, original = t[indices], target[indices], indices
    cuts = np.flatnonzero(np.abs(np.diff(target)) > 0.01) + 1
    bounds = np.r_[0, cuts, len(target)]
    return [(float(target[a]), original[a:b]) for a, b in zip(bounds[:-1], bounds[1:])]


def in_band_settling_time(t, y, ref, band, min_hold=0.5):
    """First time after which all remaining samples stay inside the band long enough."""
    inside = np.abs(y - ref) <= band
    suffix_all_inside = np.logical_and.accumulate(inside[::-1])[::-1]
    where = np.flatnonzero(suffix_all_inside)
    if len(where) and (t[-1] - t[where[0]]) >= min_hold:
        return float(t[where[0]] - t[0])
    return None


def sequence_report(t, target, y, band, steady_window, min_hold):
    """Report metrics separately for each constant-target segment and each transition."""
    segments = sequence_segments(t, target)
    lines = [
        '模式      : 多目标阶梯序列',
        '分段数    : %d' % len(segments),
        '稳态窗口  : 每段末 %.2f s' % steady_window,
        '稳态误差判据: |末尾窗口平均转速-目标| <= %.1f r/min' % band,
        '稳态波动和最大瞬时偏差单独报告；超调、带内恢复时间用于观察阶跃过程。',
        '上升/下降超调按目标变化幅值归一；首段作为起始稳态段，不计算阶跃上升时间。',
    ]

    previous_target = None
    for number, (ref, idx) in enumerate(segments, 1):
        ts, ys = t[idx], y[idx]
        if len(ts) == 0:
            continue
        duration = float(ts[-1] - ts[0])
        window = min(steady_window, max(duration, 0.01))
        tail_mask = ts >= (ts[-1] - window)
        yss = ys[tail_mask]
        mean = float(np.mean(yss))
        steady_error = mean - ref
        ripple = float(np.max(np.abs(yss - mean)))
        max_track_error = float(np.max(np.abs(yss - ref)))
        passed = abs(steady_error) <= band

        lines.append('')
        if previous_target is None:
            lines.append('段 %d: 起始目标 %.1f r/min, %.2f s' % (number, ref, duration))
            lines.append('  上升/下降时间: 首段无目标切换，N/A')
            lines.append('  稳态均值误差: %+.2f r/min [%s ±%.1f]' %
                         (steady_error, '符合' if passed else '超出', band))
            lines.append('  稳态波动: ±%.2f r/min；末段最大瞬时偏差: %.2f r/min (单独记录)' %
                         (ripple, max_track_error))
        else:
            delta = ref - previous_target
            initial = float(ys[0])
            direction = 1.0 if delta > 0 else -1.0
            q10 = initial + 0.10 * delta
            q90 = initial + 0.90 * delta
            crossed10 = np.flatnonzero(direction * (ys - q10) >= 0)
            crossed90 = np.flatnonzero(direction * (ys - q90) >= 0)
            t10 = float(ts[crossed10[0]]) if len(crossed10) else None
            t90 = float(ts[crossed90[0]]) if len(crossed90) else None
            transition_time = t90 - t10 if t10 is not None and t90 is not None else None
            if delta > 0:
                peak = float(np.max(ys))
                over = max(0.0, peak - ref) / abs(delta) * 100.0
                extreme_name = '峰值'
            else:
                peak = float(np.min(ys))
                over = max(0.0, ref - peak) / abs(delta) * 100.0
                extreme_name = '最低值'
            settle = in_band_settling_time(ts, ys, ref, band, min_hold)
            lines.append('段 %d: %.1f -> %.1f r/min, %.2f s' %
                         (number, previous_target, ref, duration))
            lines.append('  %s超调: %.1f%% (%s %.1f r/min)' %
                         ('上升' if delta > 0 else '下降侧', over, extreme_name, peak))
            lines.append('  %s时间(10%%->90%%步幅): %s' %
                         ('上升' if delta > 0 else '下降',
                          '%.3f s' % transition_time if transition_time is not None else '未跨过两个阈值'))
            lines.append('  进入并持续保持±%.1f r/min: %s' %
                         (band, '%.3f s' % settle if settle is not None else '本段内未持续进入'))
            lines.append('  稳态均值误差: %+.2f r/min [%s ±%.1f]' %
                         (steady_error, '符合' if passed else '超出', band))
            lines.append('  稳态波动: ±%.2f r/min；末段最大瞬时偏差: %.2f r/min (单独记录)' %
                         (ripple, max_track_error))

        previous_target = ref
    return '\n'.join(lines)


def disturb_metrics(t, y, ref, band):
    """Disturbance response: max deviation and return to a relative band."""
    dev = np.abs(y - ref) / (abs(ref) if ref else 1.0)
    if not np.any(dev > 0.3):
        return {'recover': None, 'max_dev': float(np.max(dev)), 'trough_t': None,
                'note': '未检测到明显扰动(偏离<30%)'}
    trough = int(np.argmax(dev))
    recover = None
    for i in range(trough, len(y)):
        if dev[i] < band:
            recover = float(t[i] - t[trough])
            break
    return {'recover': recover, 'max_dev': float(dev[trough]),
            'trough_t': float(t[trough]), 'note': None}


def main():
    ap = argparse.ArgumentParser(description='M4-3 speed-loop metrics')
    ap.add_argument('--input', required=True, help='CSV 文件路径')
    ap.add_argument('--ref', default=None, help='单目标分析的参考转速 r/min')
    ap.add_argument('--mode', choices=['step', 'disturb'], default='step')
    ap.add_argument('--sequence', action='store_true',
                    help='按 CSV 中 target_rpm 的变化分段分析 2.5 阶梯序列')
    ap.add_argument('--tol', type=float, default=0.02,
                    help='单阶跃稳态相对带，默认 0.02')
    ap.add_argument('--band', type=float, default=0.05,
                    help='抗扰模式恢复带相对值，默认 0.05')
    ap.add_argument('--track-band', type=float, default=5.0,
                    help='阶梯序列稳态跟踪带，单位 r/min，默认 ±5')
    ap.add_argument('--steady-window', type=float, default=1.0,
                    help='每段末尾用于计算稳态误差/波动的窗口秒数，默认 1 s')
    ap.add_argument('--hold-in-band', type=float, default=0.5,
                    help='进入目标带后需连续保持的秒数，默认 0.5 s')
    ap.add_argument('--out', default=None, help='把结果追加写入指定文本文件')
    args = ap.parse_args()

    if not os.path.exists(args.input):
        raise SystemExit('[错误] 找不到文件: %s' % args.input)
    t, r, y, pwm = load_csv(args.input)

    if args.sequence:
        if args.mode != 'step':
            raise SystemExit('[错误] --sequence 只能与 step 模式一起使用。')
        if args.track_band <= 0 or args.steady_window <= 0 or args.hold_in_band <= 0:
            raise SystemExit('[错误] 跟踪带、稳态窗口和带内保持时间都必须大于 0。')
        result = '文件      : %s\n' % args.input + sequence_report(
            t, r, y, args.track_band, args.steady_window, args.hold_in_band)
    else:
        if args.ref is not None:
            ref = float(args.ref)
        else:
            good = r[np.isfinite(r)]
            ref = float(np.mean(good[-max(1, len(good) // 10):])) if len(good) else 0.0
        if ref == 0:
            raise SystemExit('[错误] 参考值为 0，请用 --ref 指定。')
        lines = ['文件      : %s' % args.input, '模式      : %s' % args.mode,
                 '参考转速  : %.1f r/min' % ref,
                 '采样点数  : %d, 时长 %.2f s' % (len(t), t[-1] - t[0])]
        if args.mode == 'step':
            m = step_metrics(t, y, ref)
            lines.extend([
                '超调量    : %.1f %% (峰值 %.1f rpm)' % (m['overshoot'], m['ymax']),
                '上升时间  : %s' % ('%.3f s (10%%→90%%)' % m['rise'] if m['rise'] is not None else 'N/A(未到90%)'),
                '稳态误差  : %.2f rpm (%.1f %%)' % (m['ss_err'], m['ss_err'] / ref * 100.0),
                '稳态波动  : 峰峰值 %.2f rpm, 标准差 %.2f rpm' % (m['ss_rip'], m['ss_std'])])
        else:
            d = disturb_metrics(t, y, ref, args.band)
            lines.append('最大偏离  : %.1f %%' % (d['max_dev'] * 100.0))
            lines.append('恢复时间  : %s' % ('%.3f s (回到 ±%.0f%% 带内)' % (d['recover'], args.band * 100.0)
                                           if d['recover'] is not None else '未恢复到 ±%.0f%% 带内' % (args.band * 100.0)))
            if d['note']:
                lines.append('备注      : %s' % d['note'])
        result = '\n'.join(lines)

    print(result)
    if args.out:
        with open(args.out, 'a', encoding='utf-8') as f:
            f.write(result + '\n\n')


if __name__ == '__main__':
    main()