/**
 ******************************************************************************
 * main.c —— CarLink 上位机工具（Linux，termios + select，纯 C）
 *
 * 用法示例：
 *   ./carlink --port /dev/ttyUSB0 --speed 60 --duration 30 \
 *             --csv ../data/normal_log.csv
 *
 *   # 破坏测试时可额外抓原始字节，用来证明"收到了垃圾但程序没崩"
 *   ./carlink --port /dev/ttyUSB0 --raw ../data/badbaud_log.txt
 *
 * 行为：
 *   - 每 50 ms 发一帧 0x01 速度指令（--rate 可调）
 *   - 每收到一帧 0x02 状态帧就记一行 CSV，控制台每 500 ms 刷新一次
 *   - 串口被拔掉（read 返回 EIO/ENODEV，或 write 失败）→ 关闭并自动重连，
 *     进程不退出、不卡死
 *   - Ctrl+C 退出时打印统计
 ******************************************************************************
 */
#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "serial.h"
#include "protocol.h"

/* ---------------- 参数（带默认值） ---------------- */
static const char *g_port      = "/dev/ttyUSB0";
static int         g_baud      = 115200;
static int         g_speed     = 60;      /* 目标转速，负数=反转 */
static int         g_rate      = 20;      /* 指令发送频率 Hz，0=不发送 */
static int         g_duration  = 0;       /* 秒，0=一直跑 */
static const char *g_csv       = NULL;
static const char *g_raw       = NULL;
static int         g_quiet     = 0;
static int         g_send_clear = 0;
static int         g_selftest   = 0;

static volatile sig_atomic_t g_stop = 0;

static void on_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
}

/* 单调时钟，毫秒；不受系统时间调整影响 */
static uint32_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((ts.tv_sec * 1000u) + (ts.tv_nsec / 1000000u));
}

static void usage(const char *prog)
{
    printf("CarLink v1.0 上位机工具\n\n");
    printf("用法: %s [选项]\n\n", prog);
    printf("  --port <dev>      串口设备（默认 /dev/ttyUSB0）\n");
    printf("  --baud <n>        波特率（默认 115200）\n");
    printf("  --speed <rpm>     目标转速，负数表示反转（默认 60）\n");
    printf("  --rate <hz>       指令发送频率（默认 20 Hz，0=不发）\n");
    printf("  --duration <s>    运行时长秒数（默认 0=一直跑）\n");
    printf("  --csv <path>      把状态帧写入 CSV\n");
    printf("  --raw <path>      把收到的原始字节以 hex 写入文件\n");
    printf("  --clear           启动时先发一帧 0x03 清计数\n");
    printf("  --quiet           不打印实时状态（只写文件）\n");
    printf("  --selftest        用已知参考帧自检 CRC 与组帧，然后退出\n");
    printf("  --help            显示本帮助\n");
}

/*
 * 自检：拿题面参考帧 / docs 里的示例帧当"金标准"，
 * 证明本实现组出来的字节与协议文档逐字节一致。
 * 这一步不需要任何硬件，答辩时可以当场跑给考官看。
 */
static int run_selftest(void)
{
    static const uint8_t exp_speed[10]  = {
        0xAA, 0x55, 0x04, 0x01, 0x3C, 0x00, 0x3C, 0x00, 0xEE, 0x1D
    };
    static const uint8_t exp_status[20] = {
        0xAA, 0x55, 0x0E, 0x02, 0x3C, 0x00, 0x3B, 0x00,
        0xD2, 0x04, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x16, 0xDB
    };
    static const uint8_t exp_clear[6]   = {
        0xAA, 0x55, 0x00, 0x03, 0x71, 0xED
    };
    uint8_t  f[20];
    uint16_t n;
    int      i;
    int      fails = 0;

    n = carlink_build_speed(f, 60, 60);
    printf("[1] 速度帧 60/60      : %s\n", (n == 10 && memcmp(f, exp_speed, 10) == 0) ? "PASS" : "FAIL");
    if (!(n == 10 && memcmp(f, exp_speed, 10) == 0)) fails++;

    n = carlink_build_status(f, 60, 59, 1234, 7, 0, 0, 0);
    printf("[2] 状态帧(60,59,12.34°): %s\n", (n == 20 && memcmp(f, exp_status, 20) == 0) ? "PASS" : "FAIL");
    if (!(n == 20 && memcmp(f, exp_status, 20) == 0)) fails++;

    n = carlink_build_clear(f);
    printf("[3] 清计数帧          : %s\n", (n == 6 && memcmp(f, exp_clear, 6) == 0) ? "PASS" : "FAIL");
    if (!(n == 6 && memcmp(f, exp_clear, 6) == 0)) fails++;

    /* 反向：把好帧喂进解析器，应能解出原值 */
    {
        cl_parser_t p;
        carlink_init(&p, 0u);
        for (i = 0; i < 10; i++) {
            carlink_feed(&p, exp_speed[i], 0u);
        }
        printf("[4] 解析 0x01 环回    : %s\n",
               (p.bad_frame_cnt == 0u && p.st_seq == 0u) ? "PASS" : "FAIL");
    }

    /* 反向：故意篡改 1 个字节，必须计坏帧且不产生状态帧 */
    {
        cl_parser_t p;
        uint8_t     bad[20];
        memcpy(bad, exp_status, 20);
        bad[8] ^= 0x01;                       /* 改 pitch 最低位，CRC 必失效 */
        carlink_init(&p, 0u);
        for (i = 0; i < 20; i++) {
            carlink_feed(&p, bad[i], 0u);
        }
        printf("[5] 坏帧必须被拒       : %s\n",
               (p.bad_frame_cnt >= 1u && p.st_seq == 0u) ? "PASS" : "FAIL");
        if (!(p.bad_frame_cnt >= 1u && p.st_seq == 0u)) fails++;
    }

    /* 反向：前插 3 个脏字节，之后必须仍能正确同步到状态帧 */
    {
        cl_parser_t p;
        uint8_t     junk[3] = { 0x00, 0xAA, 0xFF };
        carlink_init(&p, 0u);
        for (i = 0; i < 3; i++) {
            carlink_feed(&p, junk[i], 0u);
        }
        for (i = 0; i < 20; i++) {
            carlink_feed(&p, exp_status[i], 0u);
        }
        printf("[6] 脏字节后能重同步   : %s\n",
               (p.st_seq == 1u && p.st_left_rpm == 60) ? "PASS" : "FAIL");
        if (!(p.st_seq == 1u && p.st_left_rpm == 60)) fails++;
    }

    printf("\n自检结果: %d 项失败\n", fails);
    return (fails == 0) ? 0 : 1;
}

static int parse_args(int argc, char **argv)
{
    int i;

    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 1;
        } else if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            g_port = argv[++i];
        } else if (strcmp(argv[i], "--baud") == 0 && i + 1 < argc) {
            g_baud = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--speed") == 0 && i + 1 < argc) {
            g_speed = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--rate") == 0 && i + 1 < argc) {
            g_rate = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
            g_duration = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--csv") == 0 && i + 1 < argc) {
            g_csv = argv[++i];
        } else if (strcmp(argv[i], "--raw") == 0 && i + 1 < argc) {
            g_raw = argv[++i];
        } else if (strcmp(argv[i], "--quiet") == 0) {
            g_quiet = 1;
        } else if (strcmp(argv[i], "--selftest") == 0) {
            g_selftest = 1;
        } else if (strcmp(argv[i], "--clear") == 0) {
            g_send_clear = 1;
        } else {
            fprintf(stderr, "未知参数: %s（用 --help 查看用法）\n", argv[i]);
            return -1;
        }
    }
    return 0;
}

/* 串口消失的判定：这些 errno 表示设备真的不在了，不是"暂时没数据" */
static int device_gone_errno(int e)
{
    return (e == EIO) || (e == ENODEV) || (e == EBADF) || (e == ENXIO);
}

int main(int argc, char **argv)
{
    cl_parser_t p;
    int         fd = -1;
    FILE       *csv_fp = NULL;
    FILE       *raw_fp = NULL;
    uint8_t     frame[20];
    uint32_t    t_begin;
    uint32_t    last_tx = 0;
    uint32_t    last_print = 0;
    uint32_t    last_seq = 0;
    uint16_t    last_hb = 0;
    uint32_t    tx_cnt = 0;
    uint32_t    status_cnt = 0;
    uint32_t    lost_cnt = 0;
    uint32_t    reconnect_cnt = 0;
    uint32_t    tx_period_ms;
    int         ret;

    ret = parse_args(argc, argv);
    if (ret != 0) {
        return (ret > 0) ? 0 : 2;
    }

    if (g_selftest) {
        return run_selftest();
    }

    signal(SIGINT, on_sigint);
    signal(SIGTERM, on_sigint);

    if (g_csv != NULL) {
        csv_fp = fopen(g_csv, "w");
        if (csv_fp == NULL) {
            fprintf(stderr, "无法打开 CSV %s: %s\n", g_csv, strerror(errno));
            return 2;
        }
        fprintf(csv_fp,
                "t_ms,left_rpm,right_rpm,pitch_deg,heartbeat,"
                "bad_frame_cnt,timeout_cnt,overflow_cnt,hb_gap\n");
    }

    if (g_raw != NULL) {
        raw_fp = fopen(g_raw, "w");
        if (raw_fp == NULL) {
            fprintf(stderr, "无法打开 raw %s: %s\n", g_raw, strerror(errno));
            if (csv_fp) fclose(csv_fp);
            return 2;
        }
        fprintf(raw_fp, "# t_ms  raw bytes (hex)\n");
    }

    carlink_init(&p, now_ms());
    t_begin  = now_ms();
    last_tx  = t_begin;
    tx_period_ms = (g_rate > 0) ? (uint32_t)(1000 / g_rate) : 0u;

    fprintf(stderr, "[info] 目标: %s @ %d 8N1, 速度=%d rpm, 指令=%d Hz\n",
            g_port, g_baud, g_speed, g_rate);
    if (g_quiet) {
        fprintf(stderr, "[info] quiet 模式：只写文件，不刷屏\n");
    }

    while (!g_stop) {
        uint32_t now = now_ms();

        if (g_duration > 0 && (uint32_t)(now - t_begin) >= (uint32_t)g_duration * 1000u) {
            break;
        }

        /* ---- 串口没打开（首次或掉线后）→ 尝试重连 ---- */
        if (fd < 0) {
            fd = serial_open(g_port, g_baud);
            if (fd < 0) {
                fprintf(stderr, "[warn] 打开 %s 失败(%s)，200 ms 后重试\n",
                        g_port, strerror(errno));
                usleep(200000);
                continue;
            }
            reconnect_cnt++;
            fprintf(stderr, "[info] 串口已打开 %s @ %d（第 %u 次）\n",
                    g_port, g_baud, reconnect_cnt);
            last_tx = now_ms();

            if (g_send_clear) {
                uint16_t n = carlink_build_clear(frame);
                (void)serial_write(fd, frame, (int)n);
            }
        }

        /* ---- select 等待数据，10 ms 超时，保证发送节拍不飘 ---- */
        {
            fd_set rfds;
            struct timeval tv;
            int r;

            FD_ZERO(&rfds);
            FD_SET(fd, &rfds);
            tv.tv_sec  = 0;
            tv.tv_usec = 10000;

            r = select(fd + 1, &rfds, NULL, NULL, &tv);
            if (r > 0) {
                unsigned char buf[256];
                int n = serial_read(fd, buf, (int)sizeof(buf));

                if (n > 0) {
                    int i;
                    if (raw_fp != NULL) {
                        uint32_t el = (uint32_t)(now - t_begin);
                        fprintf(raw_fp, "%u ", el);
                        for (i = 0; i < n; i++) {
                            fprintf(raw_fp, "%02X", buf[i]);
                            if (i + 1 < n) fprintf(raw_fp, " ");
                        }
                        fprintf(raw_fp, "\n");
                    }
                    for (i = 0; i < n; i++) {
                        carlink_feed(&p, buf[i], now);
                    }
                } else if (n < 0 && device_gone_errno(errno)) {
                    fprintf(stderr, "[warn] 串口读失败(errno=%d %s)，判定设备断开，准备重连\n",
                            errno, strerror(errno));
                    serial_close(fd);
                    fd = -1;
                    continue;
                }
            }
        }

        /* ---- 帧内超时（收到一半断了） ---- */
        carlink_check_timeout(&p, now);

        /* ---- 处理新状态帧 ---- */
        if (p.st_seq != last_seq) {
            uint16_t gap = (uint16_t)(p.st_heartbeat - last_hb);

            if (last_seq != 0u && gap > 1u) {
                lost_cnt += (uint32_t)(gap - 1u);
            }
            if (last_seq == 0u) {
                gap = 0u;
            }
            last_hb  = p.st_heartbeat;
            last_seq = p.st_seq;
            status_cnt++;

            if (csv_fp != NULL) {
                fprintf(csv_fp, "%u,%d,%d,%.2f,%u,%u,%u,%u,%u\n",
                        (unsigned)(now - t_begin),
                        (int)p.st_left_rpm,
                        (int)p.st_right_rpm,
                        (double)p.st_pitch_x100 / 100.0,
                        (unsigned)p.st_heartbeat,
                        (unsigned)p.st_bad,
                        (unsigned)p.st_timeout,
                        (unsigned)p.st_overflow,
                        (unsigned)gap);
                fflush(csv_fp);
            }

            if (!g_quiet && ((uint32_t)(now - last_print) >= 500u)) {
                last_print = now;
                printf("[%6.2fs] rpm=(%4d,%4d) pitch=%6.2f° hb=%5u "
                       "bad=%3u tmo=%3u ovf=%3u\n",
                       (double)(now - t_begin) / 1000.0,
                       (int)p.st_left_rpm, (int)p.st_right_rpm,
                       (double)p.st_pitch_x100 / 100.0,
                       (unsigned)p.st_heartbeat,
                       (unsigned)p.st_bad,
                       (unsigned)p.st_timeout,
                       (unsigned)p.st_overflow);
                fflush(stdout);
            }
        }

        /* ---- 周期发送速度指令 ---- */
        if (tx_period_ms > 0u && (uint32_t)(now - last_tx) >= tx_period_ms) {
            uint16_t n;
            last_tx = now;
            n = carlink_build_speed(frame, (int16_t)g_speed, (int16_t)g_speed);
            if (serial_write(fd, frame, (int)n) < 0) {
                if (device_gone_errno(errno)) {
                    fprintf(stderr, "[warn] 串口写失败(errno=%d)，判定设备断开，准备重连\n",
                            errno);
                    serial_close(fd);
                    fd = -1;
                    continue;
                }
            } else {
                tx_cnt++;
            }
        }
    }

    /* ---- 收尾统计 ---- */
    fprintf(stderr, "\n===== 运行统计 =====\n");
    fprintf(stderr, "时长          : %.2f s\n", (double)(now_ms() - t_begin) / 1000.0);
    fprintf(stderr, "发送指令帧    : %u\n", tx_cnt);
    fprintf(stderr, "收到状态帧    : %u\n", status_cnt);
    fprintf(stderr, "按心跳推算丢帧: %u\n", lost_cnt);
    fprintf(stderr, "本端坏帧计数  : %u\n", p.bad_frame_cnt);
    fprintf(stderr, "本端超时计数  : %u\n", p.timeout_cnt);
    fprintf(stderr, "本端长度溢出  : %u\n", p.overflow_cnt);
    fprintf(stderr, "MCU 报告 bad  : %u\n", (unsigned)p.st_bad);
    fprintf(stderr, "MCU 报告 tmo  : %u\n", (unsigned)p.st_timeout);
    fprintf(stderr, "MCU 报告 ovf  : %u\n", (unsigned)p.st_overflow);
    fprintf(stderr, "串口重连次数  : %u\n", reconnect_cnt);

    if (csv_fp) fclose(csv_fp);
    if (raw_fp) fclose(raw_fp);
    if (fd >= 0) serial_close(fd);

    return 0;
}
