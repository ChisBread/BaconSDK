#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "platform.h"
#include "common.h"

namespace bacon {
// linux spidev 实现
#ifdef __linux__
int fd;
static char spi_dev_path[256] = "/dev/spidev3.0";
static unsigned mode = SPI_MODE_3;
static uint8_t bits = 8;
static uint32_t speed = 32000000; // 设置SPI速度为48MHz
static bool lsb = false;
static uint16_t delay = 0;

void spi_init(const char *user_path, uint32_t user_speed) {
    if (user_path != NULL) {
        strcpy(spi_dev_path, user_path);
    }
    if (user_speed != 0) {
        speed = user_speed;
    }
    int ret;
    // 打开 SPI 设备
    fd = open(spi_dev_path, O_RDWR);
    if (fd < 0) {
        perror("Can't open SPI device");
        exit(1);
    }

    // 设置 SPI 工作模式
    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    if (ret == -1) {
        perror("Can't set SPI mode");
        exit(1);
    }

    // 设置位数
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1) {
        perror("Can't set bits per word");
        exit(1);
    }

    // 设置SPI速度
    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1) {
        perror("Can't set max speed");
        exit(1);
    }

    // 设置字节序
    ret = ioctl(fd, SPI_IOC_WR_LSB_FIRST, &lsb);
    if (ret == -1) {
        perror("Can't set lsb");
        exit(1);
    }

    // 打印设置
    printf("SPI mode: 0x%x\n", mode);
    printf("Bits per word: %d\n", bits);
    printf("Max speed: %d Hz\n", speed);
    printf("LSB: %s\n", lsb ? "yes" : "no");
}

void spi_close() {
    close(fd);
}

void transfer(uint8_t const *tx, uint8_t *rx, size_t len) {
    int ret;
    struct spi_ioc_transfer tr = {};
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = __u32(len);
    tr.speed_hz = speed;
    tr.delay_usecs = delay;
    tr.bits_per_word = bits;
    tr.cs_change = 0;

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

    if (ret < 1) {
        perror("SPI transfer failed");
    }
}

vecbytes transfer(const vecbytes &tx_buffer) {
    vecbytes rx_buffer(tx_buffer.size(), 0);
    transfer((uint8_t const *)(tx_buffer.data()), rx_buffer.data(), tx_buffer.size());
    return rx_buffer;
}

vecbytes transfer(const std::vector<BitArray> &commands) {
    BitArray merged = merge_cmds(commands);
    vecbytes tx_buffer = merged.bytes();
    // 如果刚好是8的倍数，需要补一个0
    if (tx_buffer.size() == merged.size()/8) {
        tx_buffer.push_back(0);
    }

    vecbytes rx_buffer(tx_buffer.size(), 0);
    transfer((uint8_t const *)(tx_buffer.data()), rx_buffer.data(), tx_buffer.size());
    return rx_buffer;
}

vecbytes transfer(const BitArray &command) {
    return transfer(std::vector<BitArray>({command}));
}

#endif // __linux__

} // namespace bacon