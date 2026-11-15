#ifndef CARLINK_SERIAL_H
#define CARLINK_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * serial —— Linux 串口封装（termios）
 *
 * 关键点（都是现场破坏测试会碰到的）：
 *   - O_NONBLOCK + VMIN=0/VTIME=0：read 永不阻塞，配合 select 使用；
 *   - raw 模式：关闭 ICANON/ECHO，不做任何字符映射，保证 0xAA/0x55 原样收到；
 *   - 关闭软/硬件流控，否则二进制数据里的 0x11/0x13 会被当成 XON/XOFF 吃掉；
 *   - 设备被拔掉时 read 返回 -1(EIO) 或 0，上层据此关闭并重连。
 */

int  serial_open(const char *dev, int baud);
void serial_close(int fd);
int  serial_read(int fd, unsigned char *buf, int max_len);
int  serial_write(int fd, const unsigned char *buf, int len);

#ifdef __cplusplus
}
#endif

#endif /* CARLINK_SERIAL_H */
