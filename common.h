#ifndef __COMMANDS_H__
#define __COMMANDS_H__

#include <cstdint>
#include <vector>
#include <string>
#include <unordered_map>

#include "bitarray.hpp"

#define ROM_MAX_SIZE 0x2000000
#define RAM_MAX_SIZE 0x20000
#define RAM_512_SIZE 0x10000

namespace bacon {

vecbytes build_cmd(const BitArray &command);
BitArray merge_cmds(const std::vector<BitArray> &commands);
vecbytes slice_by_bitidx(const vecbytes &bytes, size_t begin, size_t end);
std::string to_string(const BitArray &bits);
std::string to_string(const std::unordered_map<std::string, uint32_t> &data);
BitArray make_power_control_command(bool v3_3v, bool v5v);
BitArray make_power_read_command();
BitArray make_cart_30bit_read_command();
std::unordered_map<std::string, uint32_t> extract_cart_30bit_read_data(const vecbytes &rx_buffer);
BitArray make_cart_30bit_write_command(
    bool phi = false, bool req = false,
    // 低电平有效
    bool wr = true, bool rd = true,
    bool cs1 = true, bool cs2 = true, 
    // 小端模式
    uint16_t v16 = 0, uint8_t v8 = 0);
BitArray make_gba_wr_rd_write_command(bool wr, bool rd);
BitArray make_v16bit_data_write_command(uint16_t data, bool flip = false);
BitArray make_gba_rom_data_write_command(uint16_t data, bool flip = false);
BitArray make_rom_read_cycle_command_30bit(int times = 1);
std::vector<std::unordered_map<std::string, uint32_t>> extract_read_cycle_data_30bit(const vecbytes &data, int times = 1);
BitArray make_gba_rom_data_read_command(bool flip = false);
BitArray make_rom_read_cycle_command(int times = 1);
std::vector<uint16_t> extract_read_cycle_data(const vecbytes &data, int times = 1);
BitArray make_gba_rom_cs_write(bool cs);
BitArray make_rom_write_cycle_command_with_addr(const std::vector<std::pair<uint32_t, uint16_t>> &addrdatalist, bool hwaddr = true);
BitArray make_rom_write_cycle_command_with_addr(
    std::vector<std::pair<uint32_t, uint16_t>>::const_iterator begin, 
    std::vector<std::pair<uint32_t, uint16_t>>::const_iterator end, bool hwaddr = true);
BitArray make_rom_write_cycle_command_sequential(const std::vector<uint16_t> &datalist);
BitArray make_rom_write_cycle_command_sequential(std::vector<uint16_t>::const_iterator begin, std::vector<uint16_t>::const_iterator end);
BitArray make_gba_rom_addr_read_command();
BitArray make_ram_write_cycle_with_addr(const std::vector<std::pair<uint16_t, uint8_t>> &addrdatalist);
BitArray make_ram_write_cycle_command(uint16_t addr, const vecbytes &data);
BitArray make_ram_read_cycle_command(uint16_t addr = 0, int times = 1);
vecbytes extract_ram_read_cycle_data(const vecbytes &data, int times = 1);
vecbytes make_rom_read_cycle_command_with_cache(size_t times = 1);
vecbytes make_ram_read_cycle_command_with_cache(uint16_t addr, size_t times = 1);

uint8_t reverse_bits(uint8_t byte);
uint16_t reverse_bits_16bit(uint16_t word);
int start_low_latency(void);
void stop_low_latency(void);
} // namespace bacon

#endif // __COMMANDS_H__