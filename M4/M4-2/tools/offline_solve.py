"""离线解算未知 IMU CSV 的 pitch，并生成 CSV 与 SVG 曲线。

在 M4-2 目录运行：
    python3 tools/offline_solve.py --input imu_raw.csv --out solved.csv
若考官另给独立参考角度 CSV：
    python3 tools/offline_solve.py --input imu_raw.csv --out solved.csv --reference imu_truth.csv

IMU 输入至少含 t, ax, ay, az, gx, gy, gz；单位为秒、g、度/秒。
程序使用文件开头自动检测到的静止段估计陀螺零偏，不使用写死的偏置。
输出列至少有 t, angle_deg；SVG 曲线不需要 numpy 或 matplotlib。
"""

import argparse
import csv
import math
import statistics
from pathlib import Path
from xml.sax.saxutils import escape


ALIASES = {
    "t": ("t", "t_s", "time", "time_s"),
    "ax": ("ax", "ax_g"), "ay": ("ay", "ay_g"), "az": ("az", "az_g"),
    "gx": ("gx", "gx_dps"), "gy": ("gy", "gy_dps"), "gz": ("gz", "gz_dps"),
}
REF_ALIASES = ("angle_ref_deg", "reference_angle_deg", "true_angle_deg", "ref_angle_deg")


def arguments():
    p = argparse.ArgumentParser(description="从 IMU 原始 CSV 自动估零偏并计算 pitch。")
    p.add_argument("--input", required=True, help="考官给的 IMU 原始 CSV")
    p.add_argument("--out", "--output", dest="out", required=True, help="解算结果 CSV")
    p.add_argument("--reference", help="可选：独立真值 CSV，至少含 t 和 angle_deg")
    p.add_argument("--alpha", type=float, default=0.99, help="互补滤波陀螺权重，默认 0.99")
    p.add_argument("--gyro-axis", choices=("x", "y", "z"), default="y", help="pitch 使用的陀螺轴，默认 y")
    p.add_argument("--still-gyro-dps", type=float, default=5.0,
                   help="开头静止检测角速度上限（°/s），默认 5，可按命令行调整")
    return p.parse_args()


def read_csv(path, required, optional=()):
    """按列名读取 CSV，列顺序可变；返回数值行和实际列名映射。"""
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError(f"{path.name} 没有表头。")
        lookup = {name.strip().lower(): name for name in reader.fieldnames}
        mapping = {}
        for key, aliases in required.items():
            found = next((lookup[a] for a in aliases if a in lookup), None)
            if found is None:
                raise ValueError(f"{path.name} 缺少 {key} 列；表头为 {reader.fieldnames}")
            mapping[key] = found
        optional_mapping = {}
        for key, aliases in optional.items():
            found = next((lookup[a] for a in aliases if a in lookup), None)
            if found is not None:
                optional_mapping[key] = found
        rows = []
        for line_no, raw in enumerate(reader, start=2):
            try:
                row = {key: float(raw[col].strip()) for key, col in mapping.items()}
                row.update({key: float(raw[col].strip()) for key, col in optional_mapping.items()})
            except (AttributeError, TypeError, ValueError):
                raise ValueError(f"{path.name} 第 {line_no} 行有空值或非数字。")
            rows.append(row)
    return rows


def pitch_from_acc(row):
    """当前坐标约定：由重力在 X 方向的分量求 pitch。"""
    return math.degrees(math.atan2(-row["ax"], math.sqrt(row["ay"] ** 2 + row["az"] ** 2)))


def detect_still_prefix(rows, gyro_limit):
    """从第一条开始找连续静止段；依数据自身判断，不固定某个绝对时刻。"""
    count = 0
    for r in rows:
        a_norm = math.sqrt(r["ax"] ** 2 + r["ay"] ** 2 + r["az"] ** 2)
        g_norm = math.sqrt(r["gx"] ** 2 + r["gy"] ** 2 + r["gz"] ** 2)
        if not (0.80 <= a_norm <= 1.20 and g_norm <= gyro_limit):
            break
        count += 1
    if count < 3:
        raise ValueError("文件开头未找到至少 3 个连续静止样本；请确认开头静止，或调整 --still-gyro-dps。")
    return count


def interp(x, xs, ys):
    """线性插值参考角；超出参考时间范围时返回 None。"""
    if x < xs[0] or x > xs[-1]:
        return None
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= x:
            lo = mid
        else:
            hi = mid
    if x == xs[hi]:
        return ys[hi]
    if x == xs[lo] or xs[hi] == xs[lo]:
        return ys[lo]
    ratio = (x - xs[lo]) / (xs[hi] - xs[lo])
    return ys[lo] + ratio * (ys[hi] - ys[lo])


def write_svg(path, series, title):
    """用 Python 标准库画 SVG；可直接用浏览器打开，不需额外绘图库。"""
    width, height = 1100, 650
    left, right, top, bottom = 90, 35, 65, 75
    pw, ph = width - left - right, height - top - bottom
    all_x = [x for _, points, _ in series for x, _ in points]
    all_y = [y for _, points, _ in series for _, y in points]
    if not all_x or not all_y:
        raise ValueError("没有可画的曲线数据。")
    xmin, xmax = min(all_x), max(all_x)
    if xmax <= xmin:
        xmax = xmin + 1.0
    ymin, ymax = min(all_y), max(all_y)
    pad = max((ymax - ymin) * 0.08, 0.5)
    ymin, ymax = ymin - pad, ymax + pad

    def sx(x): return left + (x - xmin) / (xmax - xmin) * pw
    def sy(y): return top + (ymax - y) / (ymax - ymin) * ph

    colors = ["#2878b5", "#e58b36", "#777777", "#c94c4c"]
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
           '<rect width="100%" height="100%" fill="white"/>',
           f'<text x="{width/2}" y="32" text-anchor="middle" font-size="22" font-family="sans-serif">{escape(title)}</text>']
    # 网格和坐标刻度
    for i in range(7):
        y = ymin + (ymax - ymin) * i / 6
        py = sy(y)
        svg.append(f'<line x1="{left}" y1="{py:.1f}" x2="{left+pw}" y2="{py:.1f}" stroke="#dddddd"/>')
        svg.append(f'<text x="{left-12}" y="{py+5:.1f}" text-anchor="end" font-size="13" font-family="sans-serif">{y:.2f}</text>')
    for i in range(11):
        x = xmin + (xmax - xmin) * i / 10
        px = sx(x)
        svg.append(f'<line x1="{px:.1f}" y1="{top}" x2="{px:.1f}" y2="{top+ph}" stroke="#eeeeee"/>')
        svg.append(f'<text x="{px:.1f}" y="{top+ph+23}" text-anchor="middle" font-size="13" font-family="sans-serif">{x:.2f}</text>')
    svg += [f'<line x1="{left}" y1="{top+ph}" x2="{left+pw}" y2="{top+ph}" stroke="#333"/>',
            f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top+ph}" stroke="#333"/>',
            f'<text x="{left+pw/2}" y="{height-20}" text-anchor="middle" font-size="16" font-family="sans-serif">时间 t (s)</text>',
            f'<text x="22" y="{top+ph/2}" text-anchor="middle" font-size="16" font-family="sans-serif" transform="rotate(-90 22 {top+ph/2})">角度 (°)</text>']
    # 每条输入曲线按原始采样顺序连接，不做拟合。
    for idx, (label, points, color) in enumerate(series):
        coords = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in points)
        svg.append(f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="1.5"/>')
        # 每个原始采样点都标小圆点；文件很大时仍保留数据，只抽稀显示点标记。
        marker_step = max(1, len(points) // 1500)
        for x, y in points[::marker_step]:
            svg.append(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="2.1" fill="white" stroke="{color}" stroke-width="1.1"/>')
        ly = top + 20 + idx * 23
        svg.append(f'<line x1="{left+18}" y1="{ly-5}" x2="{left+48}" y2="{ly-5}" stroke="{color}" stroke-width="2"/>')
        svg.append(f'<text x="{left+55}" y="{ly}" font-size="14" font-family="sans-serif">{escape(label)}</text>')
    svg.append('</svg>')
    path.write_text("\n".join(svg), encoding="utf-8")


def main():
    args = arguments()
    if not 0.0 < args.alpha < 1.0:
        raise SystemExit("--alpha 必须在 0 和 1 之间。")
    input_path = Path(args.input).expanduser().resolve()
    output_path = Path(args.out).expanduser().resolve()
    if not input_path.is_file():
        raise SystemExit(f"找不到输入文件：{input_path}")

    required = {key: aliases for key, aliases in ALIASES.items()}
    try:
        rows = read_csv(input_path, required, {"reference": REF_ALIASES})
        if len(rows) < 3:
            raise ValueError("至少需要 3 行 IMU 样本。")
        times = [r["t"] for r in rows]
        if any(times[i] <= times[i - 1] for i in range(1, len(times))):
            raise ValueError("输入时间 t 必须严格递增，单位为秒。")
        still_count = detect_still_prefix(rows, args.still_gyro_dps)
    except ValueError as e:
        raise SystemExit(f"输入数据错误：{e}")

    # 开头静止段的角速度平均值，就是这份数据自己估出的陀螺零偏。
    bias = {axis: sum(r["g" + axis] for r in rows[:still_count]) / still_count for axis in "xyz"}
    dt_list = [times[i] - times[i - 1] for i in range(1, len(times))]
    nominal_dt = statistics.median(dt_list)

    # 若真值在原始文件中，以这些列名之一自动读取。
    ref_points = []
    if "reference" in rows[0]:
        ref_points = [(times[i] - times[0], rows[i]["reference"]) for i in range(len(rows))]
    elif args.reference:
        ref_path = Path(args.reference).expanduser().resolve()
        if not ref_path.is_file():
            raise SystemExit(f"找不到参考真值文件：{ref_path}")
        ref_required = {"t": ALIASES["t"], "angle": ("angle_deg", "angle_ref_deg", "reference_angle_deg", "true_angle_deg")}
        try:
            ref_rows = read_csv(ref_path, ref_required)
        except ValueError as e:
            raise SystemExit(f"参考真值文件错误：{e}")
        ref_t0 = ref_rows[0]["t"]
        ref_points = [(r["t"] - ref_t0, r["angle"]) for r in ref_rows]
        if any(ref_points[i][0] <= ref_points[i - 1][0] for i in range(1, len(ref_points))):
            raise SystemExit("参考真值文件的 t 也必须严格递增。")

    # 静止第一帧的重力方向作为角度初值；当前任务约定 pitch 用 Y 轴陀螺仪。
    pitch0 = pitch_from_acc(rows[0])
    fused, gyro_only = pitch0, pitch0
    prev_rate = rows[0]["g" + args.gyro_axis] - bias[args.gyro_axis]
    output = []
    for i, row in enumerate(rows):
        if i > 0:
            dt = times[i] - times[i - 1]
            rate = row["g" + args.gyro_axis] - bias[args.gyro_axis]
            avg_rate = 0.5 * (prev_rate + rate)
            gyro_only += avg_rate * dt
            acc_pitch = pitch_from_acc(row)
            alpha_dt = args.alpha ** (dt / nominal_dt)
            fused = alpha_dt * (fused + avg_rate * dt) + (1.0 - alpha_dt) * acc_pitch
            prev_rate = rate
        else:
            acc_pitch = pitch0
        solved_t = times[i] - times[0]
        entry = {"t": solved_t, "angle_deg": fused,
                 "gyro_only_deg": gyro_only, "acc_pitch_deg": acc_pitch}
        ref = interp(solved_t, [p[0] for p in ref_points], [p[1] for p in ref_points]) if ref_points else None
        if ref is not None:
            entry["reference_angle_deg"] = ref
            entry["error_deg"] = fused - ref
        output.append(entry)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["t", "angle_deg", "gyro_only_deg", "acc_pitch_deg"]
    if ref_points:
        fields += ["reference_angle_deg", "error_deg"]
    with output_path.open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in output:
            writer.writerow({k: f"{row[k]:.6f}" for k in fields})

    plot_series = [
        ("融合角 angle_deg", [(r["t"], r["angle_deg"]) for r in output], "#2878b5"),
        ("陀螺积分 gyro_only_deg", [(r["t"], r["gyro_only_deg"]) for r in output], "#e58b36"),
        ("加速度角 acc_pitch_deg", [(r["t"], r["acc_pitch_deg"]) for r in output], "#777777"),
    ]
    if ref_points:
        plot_series.append(("独立参考真值", [(r["t"], r["reference_angle_deg"]) for r in output if "reference_angle_deg" in r], "#c94c4c"))
    plot_path = output_path.with_suffix(".svg")
    write_svg(plot_path, plot_series, "IMU 离线俯仰角解算与对比")

    print(f"输入文件：{input_path}")
    print(f"解算 CSV：{output_path}")
    print(f"角度曲线：{plot_path}")
    print(f"开头静止段：{still_count} 个样本，约 {times[still_count - 1] - times[0]:.3f} 秒")
    print(f"估计零偏 (°/s)：X={bias['x']:+.4f}, Y={bias['y']:+.4f}, Z={bias['z']:+.4f}")
    print(f"使用 GYRO_{args.gyro_axis.upper()}，互补滤波 alpha={args.alpha:g}")
    if ref_points:
        errors = [abs(r["error_deg"]) for r in output if "error_deg" in r]
        if errors:
            mae = sum(errors) / len(errors)
            max_error = max(errors)
            print(f"参考点数：{len(errors)}；平均绝对误差={mae:.3f}°；最大绝对误差={max_error:.3f}°")
            print("误差判定（最大绝对误差 ≤ 5°）：" + ("通过" if max_error <= 5.0 else "未通过"))
        else:
            print("参考文件时间范围与 IMU 数据没有重叠，无法计算误差。")
    else:
        print("没有独立参考真值：已画出三条算法曲线，但无法计算对真值误差。")


if __name__ == "__main__":
    main()
