#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <time.h>

#include "bacon.h"

using namespace bacon;

uint64_t timems() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts); // 使用单调时钟
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

int test_readrom() {
    auto start = timems();
    vecbytes rom = AGBReadROM(0, ROM_MAX_SIZE);
    for(size_t i = 0; i < 10; i++) {
        printf("%02x", rom[i]);
    }
    printf("\n");
    auto cost = timems()-start;
    printf("Read ROM data cost %f seconds, speed: %fKB/s\n", cost/1000.0, ROM_MAX_SIZE / (cost/1000.0) / 1024.0);
    // write to test_out.gba
    FILE *fp = fopen("test_out.gba", "wb");
    if (fp == NULL) {
        perror("Can't open file");
        exit(1);
    }
    fwrite(rom.data(), 1, rom.size(), fp);
    fclose(fp);
    return 0;
}

int test_readram() {
    auto start = timems();
    vecbytes ram = AGBReadRAM(0, RAM_512_SIZE);
    for(size_t i = 0; i < 10; i++) {
        printf("%02x", ram[i]);
    }
    printf("\n");
    auto cost = timems()-start;
    printf("Read RAM data cost %f seconds, speed: %fKB/s\n", cost/1000.0, RAM_512_SIZE / (cost/1000.0) / 1024.0);
    // write to test_out.gba
    FILE *fp = fopen("test_out.sav", "wb");
    if (fp == NULL) {
        perror("Can't open file");
        exit(1);
    }
    fwrite(ram.data(), 1, ram.size(), fp);
    fclose(fp);
    return 0;
}



int main(int argc, char *argv[]) {
    if (argc == 0) {
        printf("Usage: %s test_readrom|test_readram\n", argv[0]);
    }
    //
    start_low_latency();
    // 初始化SPI接口
    int ret = spi_init();
    if (ret < 0) {
        printf("spi_init failed %d\n", ret);
        return ret;
    }
    // 上电,3.3v
    transfer({make_power_control_command(true, false)});
    // 读取电源状态
    vecbytes rx_buffer = transfer({make_power_read_command()});
    printf("Power status: %s\n", to_string(BitArray(rx_buffer)).c_str());
    if (strcmp(argv[1], "test_readrom") == 0) {
        test_readrom();
    } else if (strcmp(argv[1], "test_readram") == 0) {
        test_readram();
    }
    // 断电
    transfer({make_power_control_command(false, false)});
    // 读取电源状态
    rx_buffer = transfer({make_power_read_command()});
    printf("Power status: %s\n", to_string(BitArray(rx_buffer)).c_str());
    // 关闭SPI设备
    bacon::spi_close();
    stop_low_latency();
}
