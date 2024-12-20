#include "common.h"

#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "platform.h"

namespace bacon {


static uint8_t reverse_bits_table[0xFF+1] = {0};
static uint16_t reverse_bits_table_16[0xFFFF+1] = {0};


vecbytes build_cmd(const BitArray &command) {
    vecbytes tx_buffer = command.bytes();
    // 如果刚好是8的倍数，需要补一个0
    if (tx_buffer.size() == command.size()/8) {
        tx_buffer.push_back(0);
    }
    return tx_buffer;
}

BitArray merge_cmds(const std::vector<BitArray> &commands) {
    BitArray merged;
    merged.reserve(commands.size() * 8 + commands.size() - 1);
    // 两个cmd之间要加0
    for (size_t i = 0; i < commands.size(); i++) {
        merged.push_back(commands[i]);
        if (i != commands.size() - 1) {
            merged.push_back(0);
        }
    }
    return merged;
}

vecbytes slice_by_bitidx(const vecbytes &bytes, size_t begin, size_t end) {
    vecbytes ret((end - begin + 7) / 8, 0);
    size_t begin_byte = begin / 8;
    size_t end_byte = (end+7) / 8;
    size_t begin_bit = begin % 8;
    for (size_t i = begin_byte; i < begin_byte + ((end - begin + 7) / 8); i++) {
        uint8_t byte = 0;
        if (i < bytes.size()) {
            byte = bytes[i];
        }
        uint8_t next_byte = 0;
        if (i + 1 < bytes.size() && i + 1 < end_byte) {
            next_byte = bytes[i + 1];
        }
        ret[i - begin_byte] = (byte << begin_bit) | (next_byte >> (8 - begin_bit));
    }
    uint8_t padding = 8*ret.size() - (end - begin);
    if (padding > 0) {
        ret.back() &= 0xFF ^ ((1 << padding) - 1);
    }
    return ret;
}

std::string to_string(const BitArray &bits) {
    std::string str;
    for (size_t i = 0; i < bits.size(); i++) {
        str += bits.get(i) ? "1" : "0";
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


BitArray make_power_control_command(bool v3_3v, bool v5v) {
    if (v3_3v && v5v) {
        throw "Invalid power control command";
    }
    BitArray command({0, 0, 0, 1, 1});
    command.push_back(!v3_3v);
    command.push_back(v5v);
    return command;
}

BitArray make_power_read_command() {
    BitArray command({0, 0, 1, 0, 0, 0, 0});
    return command;
}

BitArray make_cart_30bit_read_command() {
    BitArray command(
        {0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 
         0, 0, 0, 0, 0, 
         0, 0, 0, 0, 0, 
         0, 0, 0, 0, 0, 
         0, 0, 0, 0, 0, 
         0, 0, 0, 0, 0});
    return command;
}

std::unordered_map<std::string, uint32_t> extract_cart_30bit_read_data(const vecbytes &rx_buffer) {
    std::unordered_map<std::string, uint32_t> data;
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

BitArray make_cart_30bit_write_command(
    bool phi, bool req,
    // 低电平有效
    bool wr, bool rd,
    bool cs1, bool cs2, 
    // 小端模式
    uint16_t v16, uint8_t v8) {
    BitArray command({0, 0, 0, 0, 1});
    command.push_back(req);
    command.push_back(cs2);
    BitArray v8bit(vecbytes{uint8_t(v8)});
    BitArray v16bit(vecbytes{uint8_t((v16 >> 8) & 0xff), uint8_t(v16 & 0xff)});
    command.push_back(v8bit);
    command.push_back(v16bit);
    command.push_back(cs1);
    command.push_back(rd);
    command.push_back(wr);
    command.push_back(phi);
    return command;
}

BitArray make_gba_wr_rd_write_command(bool wr, bool rd) {
    BitArray command({0, 0, 1, 0, 1});
    command.push_back(wr);
    command.push_back(rd);
    return command;
}

BitArray make_v16bit_data_write_command(uint16_t data, bool flip) {
    BitArray command({0, 1, 0, 0, 1});
    if (flip) {
        command = BitArray({0, 1, 1, 0, 0});
    }
    // little endian
    BitArray v16bit(vecbytes{uint8_t((data >> 8) & 0xff), uint8_t(data & 0xff)});
    command.push_back(v16bit);
    return command;
}

BitArray make_gba_rom_data_write_command(uint16_t data, bool flip) {
    return make_v16bit_data_write_command(data, flip);
}


BitArray __readcyclecmd_30bit = merge_cmds({
    make_gba_wr_rd_write_command(true, false),
    make_cart_30bit_read_command(),
    make_gba_wr_rd_write_command(true, true)});

BitArray make_rom_read_cycle_command_30bit(int times) {
    std::vector<BitArray> commands;
    for (int i = 0; i < times; i++) {
        commands.push_back(__readcyclecmd_30bit);
    }
    return merge_cmds(commands);
}

std::vector<std::unordered_map<std::string, uint32_t>> extract_read_cycle_data_30bit(const vecbytes &data, int times) {
    std::vector<std::unordered_map<std::string, uint32_t>> ret;
    if (data.size() * 8 < (__readcyclecmd_30bit.size() + 1) * times) {
        throw "data must be " + std::to_string((__readcyclecmd_30bit.size() + 1) * times) + " bytes, but got " + std::to_string(data.size() * 8);
    }
    for (size_t i = 0; i < data.size()*8; i += __readcyclecmd_30bit.size() + 1) {
        //BitArray one(databits.begin() + i + 8, databits.begin() + i + __readcyclecmd_30bit.size() + 1);
        vecbytes one = slice_by_bitidx(data, i + 8, i + __readcyclecmd_30bit.size() + 1);
        ret.push_back(extract_cart_30bit_read_data(one));
        if (ret.size() >= times) {
            break;
        }
    }
    return ret;
}

BitArray make_gba_rom_data_read_command(bool flip) {
    BitArray command({
        0, 1, 0, 1, flip,
        0, 0, 0, 0, 
        0, 0, 0, 0, 
        0, 0, 0, 0,
        0, 0, 0, 0});
    return command;
}

BitArray __readcyclecmd = merge_cmds({
    make_gba_wr_rd_write_command(true, false),
    make_gba_rom_data_read_command(true)});


BitArray make_rom_read_cycle_command(int times) {
    std::vector<BitArray> commands;
    commands.reserve(times*__readcyclecmd.size() + times - 1);
    for (int i = 0; i < times; i++) {
        commands.push_back(__readcyclecmd);
    }
    return merge_cmds(commands);
}

std::vector<uint16_t> extract_read_cycle_data(const vecbytes &data, int times) {
    std::vector<uint16_t> ret(times, 0);
    size_t idx = 0;
    for (size_t i = 0; i < data.size()*8; i += __readcyclecmd.size() + 1) {
        vecbytes one = slice_by_bitidx(data, i + 8 + 6, i + 8 + 6 + 16);
        ret[idx++] = reverse_bits_16bit(((one[1]) << 8) | (one[0]));
        if (idx >= times) {
            break;
        }
    }
    return ret;
}

BitArray make_gba_rom_cs_write(bool cs) {
    BitArray command({0, 0, 1, 1, 1});
    command.push_back(cs);
    return command;
}

BitArray make_rom_write_cycle_command_with_addr(const std::vector<std::pair<uint32_t, uint16_t>> &addrdatalist, bool hwaddr) {
    std::vector<BitArray> commands;
    for (auto &kv : addrdatalist) {
        auto addr = kv.first;
        if (!hwaddr) {
            addr = addr / 2;
        }
        commands.push_back(merge_cmds({
            make_cart_30bit_write_command(false, false, true, true, true, true, addr & 0xFFFF, (addr >> 16) & 0xFF),
            make_gba_rom_cs_write(false),
            make_gba_rom_data_write_command(kv.second, true)
        }));
    }
    return merge_cmds(commands);
}

BitArray make_rom_write_cycle_command_with_addr(
    std::vector<std::pair<uint32_t, uint16_t>>::const_iterator begin, 
    std::vector<std::pair<uint32_t, uint16_t>>::const_iterator end, bool hwaddr) {
    std::vector<BitArray> commands;
    for (auto it = begin; it != end; it++) {
        auto addr = it->first;
        if (!hwaddr) {
            addr = addr / 2;
        }
        commands.push_back(merge_cmds({
            make_cart_30bit_write_command(false, false, true, true, true, true, addr & 0xFFFF, (addr >> 16) & 0xFF),
            make_gba_rom_cs_write(false),
            make_gba_rom_data_write_command(it->second, true)
        }));
    }
    return merge_cmds(commands);
}

BitArray make_rom_write_cycle_command_sequential(const std::vector<uint16_t> &datalist) {
    std::vector<BitArray> commands;
    for (size_t i = 0; i < datalist.size(); i++) {
        commands.push_back(merge_cmds({
            make_gba_wr_rd_write_command(true, true),
            make_gba_rom_data_write_command(datalist[i], true)
        }));
    }
    return merge_cmds(commands);
}

BitArray make_rom_write_cycle_command_sequential(std::vector<uint16_t>::const_iterator begin, std::vector<uint16_t>::const_iterator end) {
    std::vector<BitArray> commands;
    for (auto it = begin; it != end; it++) {
        commands.push_back(merge_cmds({
            make_gba_wr_rd_write_command(true, true),
            make_gba_rom_data_write_command(*it, true)
        }));
    }
    return merge_cmds(commands);
}

BitArray make_gba_rom_addr_read_command() {
    BitArray command({
        0, 1, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0});
    return command;
}

BitArray make_ram_write_cycle_with_addr(const std::vector<std::pair<uint16_t, uint8_t>> &addrdatalist) {
    std::vector<BitArray> commands;
    for (auto &kv : addrdatalist) {
        commands.push_back(merge_cmds({
            make_cart_30bit_write_command(false, false, true, true, true, false, kv.first, kv.second),
            make_gba_wr_rd_write_command(false, true)
        }));
    }
    return merge_cmds(commands);
}

BitArray make_ram_write_cycle_command(uint16_t addr, const vecbytes &data) {
    std::vector<BitArray> commands;
    for (size_t i = 0; i < data.size(); i++) {
        commands.push_back(merge_cmds({
            make_cart_30bit_write_command(false, false, true, true, true, false, addr + i, data[i]),
            make_gba_wr_rd_write_command(false, true)
        }));
    }
    return merge_cmds(commands);
}

BitArray make_ram_read_cycle_command(uint16_t addr, int times) {
    std::vector<BitArray> commands;
    for (int i = 0; i < times; i++) {
        commands.push_back(merge_cmds({
            make_v16bit_data_write_command(addr + i),
            make_gba_rom_addr_read_command()
        }));
    }
    return merge_cmds(commands);
}

size_t __len_of_v16bit_write = make_v16bit_data_write_command(0).size();
size_t __len_of_v8bit_read = make_gba_rom_addr_read_command().size();
vecbytes extract_ram_read_cycle_data(const vecbytes &data, int times) {
    vecbytes ret;
    for (size_t i = 0; i < data.size()*8; i += __len_of_v16bit_write + __len_of_v8bit_read + 2) {
        vecbytes one = slice_by_bitidx(data, i + __len_of_v16bit_write + 1 + 6, i + __len_of_v16bit_write + 1 + __len_of_v8bit_read + 1 + 6);
        ret.push_back(reverse_bits(one[0]));
        if (ret.size() >= times) {
            break;
        }
    }
    return ret;
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

void init_reverse_bits_table() {
    // 初始化reverse_bits_table
    for (size_t i = 0; i <= 0xFF; i++) {
        reverse_bits_table[i] = _reverse_bits(i);
    }
    for (size_t i = 0; i <= 0xFFFF; i++) {
        reverse_bits_table_16[i] = _reverse_bits_16bit(i);
    }
}

uint8_t reverse_bits(uint8_t byte) {
    return reverse_bits_table[byte];
}

uint16_t reverse_bits_16bit(uint16_t word) {
    return reverse_bits_table_16[word];
}


std::unordered_map<size_t, vecbytes> __make_rom_read_cycle_command_cache;
vecbytes make_rom_read_cycle_command_with_cache(size_t times) {
    if (__make_rom_read_cycle_command_cache.find(times) == __make_rom_read_cycle_command_cache.end()) {
        __make_rom_read_cycle_command_cache[times] = build_cmd(make_rom_read_cycle_command(times));
    }
    return __make_rom_read_cycle_command_cache[times];
}

std::unordered_map<size_t, std::unordered_map<size_t, vecbytes>> __make_ram_read_cycle_command_cache;
vecbytes make_ram_read_cycle_command_with_cache(uint16_t addr, size_t times) {
    if (__make_ram_read_cycle_command_cache.find(addr) == __make_ram_read_cycle_command_cache.end()) {
        __make_ram_read_cycle_command_cache[addr] = std::unordered_map<size_t, vecbytes>();
    }
    if (__make_ram_read_cycle_command_cache[addr].find(times) == __make_ram_read_cycle_command_cache[addr].end()) {
        __make_ram_read_cycle_command_cache[addr][times] = build_cmd(make_ram_read_cycle_command(addr, times));
    }
    return __make_ram_read_cycle_command_cache[addr][times];
}

void init_rom_read_cycle_command_cache(size_t spi_buffer_size) {
    static int MAX_TIMES = spi_buffer_size*8 / (make_rom_read_cycle_command().size() + 1) - 1;
    for (size_t i = 1; i <= MAX_TIMES; i++) {
        make_rom_read_cycle_command_with_cache(i);
    }
}

void init_ram_read_cycle_command_cache(size_t spi_buffer_size) {
    static int MAX_TIMES = spi_buffer_size*8 / (make_ram_read_cycle_command(0).size() + 1) - 1;
    for (size_t i = 0; i < RAM_512_SIZE; i+=MAX_TIMES) {
        make_ram_read_cycle_command_with_cache(i, MAX_TIMES);
    }
}

int init_all_cache(size_t spi_buffer_size) {
    init_reverse_bits_table();
    init_rom_read_cycle_command_cache(spi_buffer_size);
    init_ram_read_cycle_command_cache(spi_buffer_size);
    return 0;
}

int __init_all_cache = init_all_cache(SPI_BUFFER_SIZE);

static int pm_qos_fd = -1;
int start_low_latency(void) {
    uint32_t target = 0;
    if (pm_qos_fd >= 0) return -1;
    pm_qos_fd = open("/dev/cpu_dma_latency", O_RDWR);
    if (pm_qos_fd < 0) {
        fprintf(stderr, "Failed to open PM QOS file: %s",
        strerror(errno));
        exit(errno);
    }
    return write(pm_qos_fd, &target, sizeof(target));
}
void stop_low_latency(void) {
    if (pm_qos_fd >= 0)
    close(pm_qos_fd);
}

} // namespace bacon