// bacon sdk
// Author: ChisBread (github.com/ChisBread)

// 命令结束后需要跟一个0bit
// | Command Value (5 bits) | Command Name | Description | Input Bits | Input Description | Output Bits | Output Description |
// |------------------------|--------------|-------------|------------|-------------------|-------------|--------------------|
// | 00001                 | CART_30BIT_WRITE  | 30位数据写入 | 30         | 30位数据 | -           | 无返回 |
// | 00010                 | CART_30BIT_READ   | 30位数据读取 | -          | 无输入 | 30          | 30位数据 |
// | 00011                 | MOD_WRITE    | 控制电源     | 2          | 10: 都不启用<br>00: 启用3.3v(GBA)<br>11: 启用5v(GB)<br>01: 无效 | -           | 无返回 |
// | 00100                 | MOD_READ     | 读取电源状态 | -          | 无输入 | 2           | 0: 都不启用<br>01: 启用3.3v(GBA)<br>10: 启用5v(GB)<br>11: 无效 |
// | 00101                 | GBA_WR/RD_WRITE | GBA 2bit寄存器操作，每次上升沿会使锁存的16bit地址自增1 | 2          | 0: 无效<br>01: 启用WR<br>10: 启用RD<br>11: 默认 | -           | 无返回 |
// | 00110                 | GBA_WR/RD_READ | 读取GBA 2bit寄存器状态 | -          | 无输入 | 2           | 0: 无效<br>01: 启用WR<br>10: 启用RD<br>11: 默认 |
// | 00111                 | GBA_ROM_EN_WRITE | GBA ROM使能 | 1         | 使能 | -           | 无返回 |
// | 01000                 | GBA_ROM_ADDR_READ | 读取GBA R高8位地址 | -          | 无输入 | 8          | 8位数据 |
// | 01001                 | GBA_ROM_DATA_WRITE | GBA ROM写16位数据 | 16         | 16位数据 | -           | 无返回 |
// | 01010                 | GBA_ROM_DATA_READ | 读取GBA ROM数据 | -          | 无输入 | 16          | 16位数据 |
// | 01011                 | GBA_ROM_DATA_READ_FLIP | 读取GBA ROM数据 | -          | 无输入 | 16          | 16位数据 |
// | 01100                 | GBA_ROM_DATA_WRITE_FLIP | GBA ROM写16位数据 | 16         | 16位数据 | -           | 无返回 |
// | 10000 - 11111         | RESERVED     | 预留命令     | -          | -                 | -           | -                  |

#include <vector>
#include <boost/dynamic_bitset.hpp>
#include <string>
#include <iostream>
#include <unordered_map>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

typedef boost::dynamic_bitset<uint8_t> BitArray;

int fd;
static char spi_dev_path[256] = "/dev/spidev3.0";
static unsigned mode = SPI_MODE_3;
static uint8_t bits = 8;
static uint32_t speed = 48000000; // 设置SPI速度为48MHz
static bool lsb = false;
static uint16_t delay = 0;

static uint8_t tx_buffer[4096] = {0};
static uint8_t rx_buffer[4096] = {0};

static uint8_t reverse_bits_table[256] = {0};
static uint16_t reverse_bits_table_16[65536] = {0};

void set_speed(uint32_t speed) {
    speed = speed;
}

uint8_t _reverse_bits(uint8_t byte) {
    uint8_t reversed = 0;
    for (size_t i = 0; i < 8; i++) {
        reversed |= ((byte >> i) & 1) << (7 - i);
    }
    return reversed;
}

uint16_t _reverse_bits_16bit(uint16_t word) {
    uint16_t reversed = 0;
    for (size_t i = 0; i < 16; i++) {
        reversed |= ((word >> i) & 1) << (15 - i);
    }
    return reversed;
}

uint8_t reverse_bits(uint8_t byte) {
    return reverse_bits_table[byte];
}

uint16_t reverse_bits_16bit(uint16_t word) {
    return reverse_bits_table_16[word];
}

void spi_init(void) {
    // 初始化reverse_bits_table
    for (size_t i = 0; i < 0xFF; i++) {
        reverse_bits_table[i] = _reverse_bits(i);
    }
    for (size_t i = 0; i < 0xFFFF; i++) {
        reverse_bits_table_16[i] = _reverse_bits_16bit(i);
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

std::vector<bool> merge_cmds(const std::vector<std::vector<bool>> &commands) {
    std::vector<bool> merged;
    merged.reserve(commands.size() * 8 + commands.size() - 1);
    // 两个cmd之间要加0
    for (size_t i = 0; i < commands.size(); i++) {
        merged.insert(merged.end(), commands[i].begin(), commands[i].end());
        if (i != commands.size() - 1) {
            merged.push_back(0);
        }
    }
    return merged;
}

std::vector<bool> bytes2bits(const std::vector<uint8_t> &bytes) {
    std::vector<bool> bits;
    bits.reserve(bytes.size() * 8);
    for (size_t i = 0; i < bytes.size(); i++) {
        for (size_t j = 0; j < 8; j++) {
            bits.push_back((bytes[i] >> (7 - j)) & 1);
        }
    }
    return bits;
}

std::vector<uint8_t> bits2bytes(const std::vector<bool> &bits) {
    std::vector<uint8_t> ret;
    ret.reserve(bits.size() / 8 + 1);
    for (size_t i = 0; i < bits.size(); i += 8) {
        uint8_t byte = 0;
        // 高位在前
        for (size_t j = 0; j < 8; j++) {
            if (i + j >= bits.size()) {
                byte |= 0 << (7 - j);
            } else {
                byte |= bits[i + j] << (7 - j);
            }
        }
        ret.push_back(byte);
    }
    return ret;
}

std::string to_string(const std::vector<bool> &bits) {
    std::string str;
    for (size_t i = 0; i < bits.size(); i++) {
        str += bits[i] ? "1" : "0";
    }
    return str;
}


std::string to_string(const std::unordered_map<std::string, uint32_t> &data) {
    std::string str;
    for (auto &kv : data) {
        str += kv.first + ": " + std::to_string(kv.second) + ", ";
    }
    return str;
}

void transfer(int fd, uint8_t const *tx, uint8_t *rx, size_t len) {
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

std::vector<uint8_t> transfer(int fd, const std::vector<std::vector<bool>> &commands) {
    std::vector<bool> merged = merge_cmds(commands);
    std::vector<uint8_t> tx_buffer = bits2bytes(merged);
    // 如果刚好是8的倍数，需要补一个0
    if (tx_buffer.size() == merged.size()/8) {
        tx_buffer.push_back(0);
    }

    std::vector<uint8_t> rx_buffer(tx_buffer.size(), 0);
    transfer(fd, (uint8_t const *)(tx_buffer.data()), rx_buffer.data(), tx_buffer.size());
    return rx_buffer;
}

std::vector<uint8_t> transfer(int fd, const std::vector<bool> &command) {
    return transfer(fd, std::vector<std::vector<bool>>({command}));
}

///////// C++ Interface /////////
std::vector<bool> make_power_control_command(bool v3_3v, bool v5v) {
    if (v3_3v && v5v) {
        throw "Invalid power control command";
    }
    std::vector<bool> command({0, 0, 0, 1, 1});
    command.push_back(!v3_3v);
    command.push_back(v5v);
    return command;
}

std::vector<bool> make_power_read_command() {
    std::vector<bool> command({0, 0, 1, 0, 0, 0, 0});
    return command;
}

std::vector<bool> make_cart_30bit_read_command() {
    std::vector<bool> command(
        {0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 
         0, 0, 0, 0, 0, 
         0, 0, 0, 0, 0, 
         0, 0, 0, 0, 0, 
         0, 0, 0, 0, 0, 
         0, 0, 0, 0, 0});
    return command;
}

std::unordered_map<std::string, uint32_t> extract_cart_30bit_read_data(const std::vector<uint8_t> &rx_buffer) {
    std::unordered_map<std::string, uint32_t> data;
    if (rx_buffer.size() != 5) {
        throw "data must be 5 bytes, but got " + std::to_string(rx_buffer.size());
    }
    data["phi"] = bool(rx_buffer[0] >> 1 & 1);
    data["wr"] = bool(rx_buffer[0] & 1);
    data["rd"] = bool(rx_buffer[1] >> 7);
    data["cs1"] = bool(rx_buffer[1] >> 6 & 1);
    uint16_t v16bitB0 = ((rx_buffer[1] & 0b00111111) << 2) | (rx_buffer[2] >> 6);
    uint16_t v16bitB1 = ((rx_buffer[2] & 0b00111111) << 2) | (rx_buffer[3] >> 6);
    uint8_t v8bit = ((rx_buffer[3] & 0b00111111) << 2) | (rx_buffer[4] >> 6);
    v16bitB0 = reverse_bits(v16bitB0);
    v16bitB1 = reverse_bits(v16bitB1);
    v8bit = reverse_bits(v8bit);
    data["v16bit"] = reverse_bits_16bit((v16bitB1 << 8) | v16bitB0);
    data["v8bit"] = v8bit;
    data["cs2"] = bool(rx_buffer[4] >> 5 & 1);
    data["req"] = bool(rx_buffer[4] >> 4 & 1);
    return data;
}

std::vector<bool> make_cart_30bit_write_command(
    bool phi = false, bool req = false,
    // 低电平有效
    bool wr = true, bool rd = true,
    bool cs1 = true, bool cs2 = true, 
    // 小端模式
    uint16_t v16 = 0, uint8_t v8 = 0) {
    std::vector<bool> command({0, 0, 0, 0, 1});
    command.push_back(req);
    command.push_back(cs2);
    std::vector<bool> v8bit = bytes2bits({uint8_t(v8)});
    std::vector<bool> v16bit = bytes2bits({uint8_t((v16 >> 8) & 0xff), uint8_t(v16 & 0xff)});
    command.insert(command.end(), v8bit.begin(), v8bit.end());
    command.insert(command.end(), v16bit.begin(), v16bit.end());
    command.push_back(cs1);
    command.push_back(rd);
    command.push_back(wr);
    command.push_back(phi);
    return command;
}

std::vector<bool> make_gba_wr_rd_write_command(bool wr, bool rd) {
    std::vector<bool> command({0, 0, 1, 0, 1});
    command.push_back(wr);
    command.push_back(rd);
    return command;
}

std::vector<bool> make_v16bit_data_write_command(uint16_t data, bool flip = false) {
    std::vector<bool> command({0, 1, 0, 0, 1});
    if (flip) {
        command = std::vector<bool>({0, 1, 1, 0, 0});
    }
    // little endian
    std::vector<bool> v16bit = bytes2bits({uint8_t(data & 0xff), uint8_t((data >> 8) & 0xff)});
    command.insert(command.end(), v16bit.begin(), v16bit.end());
    return command;
}

std::vector<bool> make_gba_rom_data_write_command(uint16_t data, bool flip = false) {
    return make_v16bit_data_write_command(data, flip);
}


std::vector<bool> __readcyclecmd_30bit = merge_cmds({
    make_gba_wr_rd_write_command(true, false),
    make_cart_30bit_read_command(),
    make_gba_wr_rd_write_command(true, true)});

std::vector<bool> make_rom_read_cycle_command_30bit(int times = 1) {
    std::vector<std::vector<bool>> commands;
    for (int i = 0; i < times; i++) {
        commands.push_back(__readcyclecmd_30bit);
    }
    return merge_cmds(commands);
}

std::vector<std::unordered_map<std::string, uint32_t>> extract_read_cycle_data_30bit(const std::vector<uint8_t> &data, int times = 1) {
    std::vector<std::unordered_map<std::string, uint32_t>> ret;
    if (data.size() * 8 < (__readcyclecmd_30bit.size() + 1) * times) {
        throw "data must be " + std::to_string((__readcyclecmd_30bit.size() + 1) * times) + " bytes, but got " + std::to_string(data.size() * 8);
    }
    std::vector<bool> databits = bytes2bits(data);
    for (size_t i = 0; i < databits.size(); i += __readcyclecmd_30bit.size() + 1) {
        std::vector<bool> one(databits.begin() + i + 8, databits.begin() + i + __readcyclecmd_30bit.size() + 1);
        ret.push_back(extract_cart_30bit_read_data(bits2bytes(one)));
        if (ret.size() >= times) {
            break;
        }
    }
    return ret;
}

std::vector<bool> make_gba_rom_data_read_command(bool flip = false) {
    std::vector<bool> command({
        0, 1, 0, 1, flip,
        0, 0, 0, 0, 
        0, 0, 0, 0, 
        0, 0, 0, 0,
        0, 0, 0, 0});
    return command;
}

uint16_t extract_gba_rom_read_data(const std::vector<uint8_t> &data) {
    if (data.size() < 3) {
        throw "data must be 3 bytes, but got " + std::to_string(data.size());
    }
    // printf("Data: 0b%s 0b%s 0b%s\n", to_string(bytes2bits({data[0]})).c_str(), to_string(bytes2bits({data[1]})).c_str(), to_string(bytes2bits({data[2]})).c_str());
    // printf("Data: 0x%04x\n", reverse_bits_16bit(((data[1] << 6 | data[2] >> 2) << 8) | (data[0] << 6 | data[1] >> 2)));
    return reverse_bits_16bit(((data[1] << 6 | data[2] >> 2) << 8) | (data[0] << 6 | data[1] >> 2));
}

std::vector<bool> __readcyclecmd = merge_cmds({
    make_gba_wr_rd_write_command(true, false),
    make_gba_rom_data_read_command(true)});

std::vector<bool> make_rom_read_cycle_command(int times = 1) {
    std::vector<std::vector<bool>> commands;
    for (int i = 0; i < times; i++) {
        commands.push_back(__readcyclecmd);
    }
    return merge_cmds(commands);
}

std::vector<uint16_t> extract_read_cycle_data(const std::vector<uint8_t> &data, int times = 1) {
    std::vector<uint16_t> ret;
    if (data.size() * 8 < (__readcyclecmd.size() + 1) * times) {
        throw "data must be " + std::to_string((__readcyclecmd.size() + 1) * times) + " bytes, but got " + std::to_string(data.size() * 8);
    }
    std::vector<bool> databits = bytes2bits(data);
    for (size_t i = 0; i < databits.size(); i += __readcyclecmd.size() + 1) {
        std::vector<bool> one(databits.begin() + i + 8, databits.begin() + i + __readcyclecmd.size() + 1);
        ret.push_back(extract_gba_rom_read_data(bits2bytes(one)));
        if (ret.size() >= times) {
            break;
        }
    }
    return ret;
}

std::vector<bool> make_gba_rom_cs_write(bool cs) {
    std::vector<bool> command({0, 0, 1, 1, 1});
    command.push_back(cs);
    return command;
}

std::vector<bool> make_rom_write_cycle_command_with_addr(const std::vector<std::pair<uint32_t, uint16_t>> &addrdatalist) {
    std::vector<std::vector<bool>> commands;
    for (auto &kv : addrdatalist) {
        commands.push_back(merge_cmds({
            make_cart_30bit_write_command(false, false, true, true, true, true, kv.first & 0xFFFF, (kv.first >> 16) & 0xFF),
            make_gba_rom_cs_write(false),
            make_gba_rom_data_write_command(kv.second, true)
        }));
    }
    return merge_cmds(commands);
}

std::vector<bool> make_rom_write_cycle_command_sequential(const std::vector<uint16_t> &datalist) {
    std::vector<std::vector<bool>> commands;
    for (auto &data : datalist) {
        commands.push_back(merge_cmds({
            make_gba_wr_rd_write_command(true, true),
            make_gba_rom_data_write_command(data, true)
        }));
    }
    return merge_cmds(commands);
}

std::vector<bool> make_gba_rom_addr_read_command() {
    std::vector<bool> command({
        0, 1, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0});
    return command;
}

uint8_t extract_gba_rom_addr_read_data(const std::vector<uint8_t> &data) {
    if (data.size() != 2) {
        throw "data must be 2 bytes, but got " + std::to_string(data.size());
    }
    return reverse_bits((data[0] << 6) | (data[1] >> 2));
}

std::vector<bool> make_ram_write_cycle_with_addr(const std::vector<std::pair<uint16_t, uint8_t>> &addrdatalist) {
    std::vector<std::vector<bool>> commands;
    for (auto &kv : addrdatalist) {
        commands.push_back(merge_cmds({
            make_cart_30bit_write_command(false, false, true, true, true, false, kv.first, kv.second),
            make_gba_wr_rd_write_command(false, true)
        }));
    }
    return merge_cmds(commands);
}

std::vector<bool> make_ram_write_cycle_command(uint16_t addr, const std::vector<uint8_t> &data) {
    std::vector<std::vector<bool>> commands;
    for (size_t i = 0; i < data.size(); i++) {
        commands.push_back(merge_cmds({
            make_cart_30bit_write_command(false, false, true, true, true, false, addr + i, data[i]),
            make_gba_wr_rd_write_command(false, true)
        }));
    }
    return merge_cmds(commands);
}

std::vector<bool> make_ram_read_cycle_command(uint16_t addr = 0, int times = 1) {
    std::vector<std::vector<bool>> commands;
    for (int i = 0; i < times; i++) {
        commands.push_back(merge_cmds({
            make_v16bit_data_write_command(addr + i),
            make_gba_rom_addr_read_command()
        }));
    }
    return merge_cmds(commands);
}

size_t __len_of_v16bit_write = make_v16bit_data_write_command(0).size();
size_t __len_of_v8bit_write = make_gba_rom_addr_read_command().size();
std::vector<uint8_t> extract_ram_read_cycle_data(const std::vector<uint8_t> &data, int times = 1) {
    std::vector<uint8_t> ret;
    std::vector<bool> databits = bytes2bits(data);
    for (size_t i = 0; i < databits.size(); i += __len_of_v16bit_write + __len_of_v8bit_write + 2) {
        std::vector<bool> one(databits.begin() + i + __len_of_v16bit_write + 1, databits.begin() + i + __len_of_v16bit_write + 1 + __len_of_v8bit_write + 1);
        ret.push_back(extract_gba_rom_addr_read_data(bits2bytes(one)));
        if (ret.size() >= times) {
            break;
        }
    }
    return ret;
}

///////// C++ Interface /////////

///////// C Interface /////////

///////// C Interface /////////

int main(int argc, char *argv[])
{

    // 初始化SPI接口
    spi_init();
    // 上电,3.3v
    transfer(fd, {make_power_control_command(true, false)});
    // 读取电源状态
    std::vector<uint8_t> rx_buffer = transfer(fd, {make_power_read_command()});
    printf("Power status: %s\n", to_string(bytes2bits(rx_buffer)).c_str());

    // 写30位数据
    transfer(fd, {make_cart_30bit_write_command(false, false, true, true, true, true, 0, 0)});
    transfer(fd, {make_gba_rom_cs_write(false)});
    // 读取30位数据
    rx_buffer = transfer(fd, {make_cart_30bit_read_command()});
    printf("30bit data: %s\n", to_string(bytes2bits(rx_buffer)).c_str());
    printf("Extracted data: %s\n", to_string(extract_cart_30bit_read_data(rx_buffer)).c_str());
    // 一次读取2bytesx100，一共读取32MBytes
        time_t start = time(NULL);
    for (int i = 0; i < 1024 * 1024 / 2; i+=0xFF) {
        std::vector<uint8_t> data = transfer(fd, make_rom_read_cycle_command(0xFF));
        std::vector<uint16_t> v16bitdata = extract_read_cycle_data(data, 0xFF);
        if (i % 0xFFFF == 0) {
            printf("Speed: %f KB/s; 0x%04x at 0x%04x\n", ((i * 2)/1024) / ((time(NULL) - start)*1.0), v16bitdata[0], i);
        }
    }
    // 关闭SPI设备
    close(fd);

    return 0;
}


