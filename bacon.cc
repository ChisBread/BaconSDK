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

#include <thread>
#include <mutex>
#include <list>
#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>
#include <algorithm>

#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "bacon.h"

namespace bacon {

///////// C++ Interface /////////

void ResetChip() {
    transfer({make_cart_30bit_write_command(false, false, true, true, true, true, 0, 0)});
}


vecbytes AGBReadROM(uint32_t addr, uint32_t size, bool hwaddr, bool reset) {
    static int MAX_TIMES = SPI_BUFFER_SIZE*8 / (make_rom_read_cycle_command().size() + 1) - 1;
    // prepare chip
    // to halfword
    if (!hwaddr) {
        addr = addr / 2;
        size = size / 2;
    }
    transfer({
        make_cart_30bit_write_command(false, false, true, true, true, true, addr & 0xFFFF, (addr >> 16) & 0xFF),
        make_gba_rom_cs_write(false)
    });
    size_t lowaddr = addr & 0xFFFF;
    size_t highaddr = (addr >> 16) & 0xFF;
    // if lowaddr+1 == 0x10000, highaddr+1, and reset lowaddr
    // prepare WriteRead stream
    vecbytes readbytes;
    int cycle_times = 0;
    for (int i = 0; i < size; i++) {
        cycle_times += 1;
        lowaddr += 1;
        if (lowaddr == 0x10000) {
            if (cycle_times > 0) {
                vecbytes ret = transfer(make_rom_read_cycle_command_with_cache(cycle_times));
                std::vector<uint16_t> exteds = extract_read_cycle_data(ret, cycle_times);
                for (auto &exted : exteds) {
                    readbytes.push_back(exted >> 8);
                    readbytes.push_back(exted & 0xFF);
                }
                cycle_times = 0;
            }
            highaddr += 1;
            lowaddr = 0;
            if (highaddr <= 0xFF) {
                // cs1 re-falling?
                transfer({make_cart_30bit_write_command(false, false, true, true, false, true, 0, highaddr)});
            }
        }
        if (cycle_times == MAX_TIMES || i == size - 1 && cycle_times > 0) {
            vecbytes ret = transfer(make_rom_read_cycle_command_with_cache(cycle_times));
            std::vector<uint16_t> exteds = extract_read_cycle_data(ret, cycle_times);
            for (auto &exted : exteds) {
                readbytes.push_back(exted >> 8);
                readbytes.push_back(exted & 0xFF);
            }
            cycle_times = 0;
        }
    }
    // reset chip
    if (reset) {
        transfer({make_cart_30bit_write_command(false, false, true, true, true, true, 0, 0)});
    }
    return readbytes;
}

// AGBWriteROM 传入地址是byte地址
void AGBCartWriteROMSequential(uint32_t addr, const std::vector<uint16_t> &data, bool hwaddr, bool reset) {
    static int MAX_TIMES = SPI_BUFFER_SIZE*8 / (make_rom_write_cycle_command_sequential({0}).size() + 1) - 1;
    if (!hwaddr) { // if not hwaddr, addr is byte addr
        addr = addr / 2;
    }
    transfer({
        make_cart_30bit_write_command(false, false, true, true, true, true, addr & 0xFFFF, (addr >> 16) & 0xFF),
        make_gba_rom_cs_write(false)
    });
    size_t lowaddr = addr & 0xFFFF;
    size_t highaddr = (addr >> 16) & 0xFF;
    int cycle_times = 0;
    for (size_t i = 0; i < data.size(); i++) {
        cycle_times += 1;
        lowaddr += 1;
        if (lowaddr == 0x10000) {
            if (cycle_times > 0) {
                transfer(make_rom_write_cycle_command_sequential(data.begin() + i + 1 - cycle_times, data.begin() + i + 1));
            }
            highaddr += 1;
            lowaddr = 0;
            if (highaddr <= 0xFF) {
                // cs1 re-falling?
                transfer({make_cart_30bit_write_command(false, false, true, true, false, true, 0, highaddr)});
            }
        }
        if (cycle_times == MAX_TIMES || i == data.size() - 1 && cycle_times > 0) {
            transfer(make_rom_write_cycle_command_sequential(data.begin() + i + 1 - cycle_times, data.begin() + i + 1));
        }
    }
    if (reset) {
        transfer({make_cart_30bit_write_command(false, false, true, true, true, true, 0, 0)});
    }
}

// AGBWriteROMWithAddress 传入地址是byte地址
void AGBWriteROMWithAddress(const std::vector<std::pair<uint32_t, uint16_t>> &commands, bool hwaddr) {
    static int MAX_TIMES = SPI_BUFFER_SIZE*8 / (make_rom_write_cycle_command_with_addr({{0, 0}}).size() + 1) - 1;
    int cycle_times = 0;
    for (size_t i = 0; i < commands.size(); i++) {
        // transfer({make_rom_write_cycle_command_with_addr({commands[i]}, hwaddr)});
        if (cycle_times == MAX_TIMES || i == commands.size() - 1) {
            transfer(make_rom_write_cycle_command_with_addr({commands.begin() + i + 1 - cycle_times, commands.begin() + i + 1}, hwaddr));
            cycle_times = 0;
        }
    }
    // reset chip
    transfer({make_cart_30bit_write_command(false, false, true, true, true, true, 0, 0)});
}

vecbytes AGBReadRAM(uint16_t addr, uint32_t size, bool reset) {
    static int MAX_TIMES = SPI_BUFFER_SIZE*8 / (make_ram_read_cycle_command(0).size() + 1) - 1;
    transfer({make_cart_30bit_write_command(false, false, true, false, true, false, addr, 0)});
    vecbytes readbytes;
    uint16_t start_addr = addr;
    int cycle_times = 0;
    for (int i = 0; i < size; i++) {
        cycle_times += 1;
        if (cycle_times == MAX_TIMES || i == size - 1 && cycle_times > 0) {
            vecbytes ret = transfer(make_ram_read_cycle_command_with_cache(start_addr, cycle_times));
            vecbytes exteds = extract_ram_read_cycle_data(ret, cycle_times);
            readbytes.insert(readbytes.end(), exteds.begin(), exteds.end());
            start_addr += cycle_times;
            cycle_times = 0;
        }
    }
    if (reset) {
        transfer({make_cart_30bit_write_command(false, false, true, true, true, true, 0, 0)});
    }
    return readbytes;
}

///////// C++ Interface /////////

} // namespace bacon

///////// C Interface /////////

void reset_chip() {
    bacon::ResetChip();
}

void agb_read_rom(uint32_t addr, uint32_t size, bool hwaddr, bool reset, uint8_t *rx_buffer) {
    bacon::vecbytes ret = bacon::AGBReadROM(addr, size, hwaddr, reset);
    std::copy(ret.begin(), ret.end(), rx_buffer);
}

void agb_write_rom_sequential(uint32_t addr, const uint16_t *data, size_t size, bool hwaddr, bool reset) {
    std::vector<uint16_t> data_vec(data, data + size);
    bacon::AGBCartWriteROMSequential(addr, data_vec, hwaddr, reset);
}

void agb_write_rom_with_address(const std::pair<uint32_t, uint16_t> *commands, size_t size, bool hwaddr) {
    std::vector<std::pair<uint32_t, uint16_t>> commands_vec(commands, commands + size);
    bacon::AGBWriteROMWithAddress(commands_vec, hwaddr);
}

void agb_read_ram(uint16_t addr, uint32_t size, bool reset, uint8_t *rx_buffer) {
    bacon::vecbytes ret = bacon::AGBReadRAM(addr, size, reset);
    std::copy(ret.begin(), ret.end(), rx_buffer);
}

///////// C Interface /////////