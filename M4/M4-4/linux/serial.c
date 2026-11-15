#include "serial.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

static int baud_to_speed(int baud)
{
    switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    default:     return -1;
    }
}

int serial_open(const char *dev, int baud)
{
    int fd;
    int spd;
    struct termios tio;

    spd = baud_to_speed(baud);
    if (spd < 0) {
        fprintf(stderr, "[serial] 不支持的波特率: %d\n", baud);
        return -1;
    }

    /* O_NONBLOCK：read 立刻返回，由 select 决定何时读，主循环永不被卡住 */
    fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        return -1;
    }

    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd, &tio) != 0) {
        close(fd);
        return -1;
    }

    /* 标准 raw 模式：关闭输入翻译、行缓冲、回显与信号字符（Ctrl-C 不再写进串口） */
    cfmakeraw(&tio);

    cfsetospeed(&tio, (speed_t)spd);
    cfsetispeed(&tio, (speed_t)spd);

    /* 8N1 + 无流控 + 本地连接 + 允许接收 */
    tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    tio.c_cflag |= (CS8 | CLOCAL | CREAD);

    /* 输入：关闭软件流控和所有字符转换，保证二进制字节原样 */
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_iflag &= ~(INLCR | ICRNL | IGNCR | ISTRIP | PARMRK | BRKINT | IGNBRK);

    /* 本地：raw 模式 */
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG | IEXTEN);

    /* 输出：不做换行/后处理 */
    tio.c_oflag &= ~OPOST;

    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        close(fd);
        return -1;
    }

    /* 清掉打开瞬间残留的字节，避免把上一次会话的半帧当成本次数据 */
    tcflush(fd, TCIOFLUSH);

    return fd;
}

void serial_close(int fd)
{
    if (fd >= 0) {
        close(fd);
    }
}

int serial_read(int fd, unsigned char *buf, int max_len)
{
    /* 非阻塞：没数据时返回 0，设备消失时返回 -1 并把 errno 留给上层判断 */
    return (int)read(fd, buf, (size_t)max_len);
}

int serial_write(int fd, const unsigned char *buf, int len)
{
    int done = 0;

    while (done < len) {
        int w = (int)write(fd, buf + done, (size_t)(len - done));
        if (w < 0) {
            if ((errno == EAGAIN) || (errno == EWOULDBLOCK)) {
                usleep(1000);
                continue;
            }
            return -1;
        }
        done += w;
    }
    return done;
}
