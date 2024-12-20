#ifndef __BACON_H__
#define __BACON_H__
#include <cstdint>

#include "common.h"
#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

void reset_chip();
int power_control(bool v3_3v, bool v5v);
void agb_read_rom(uint32_t addr, uint32_t size, bool hwaddr, bool reset, uint8_t *rx_buffer);
void agb_write_rom_sequential(uint32_t addr, const uint16_t *data, size_t size, bool hwaddr, bool reset);
void agb_write_rom_with_address(uint32_t *addrs, uint16_t *datas, size_t size, bool hwaddr);
void agb_read_ram(uint16_t addr, uint32_t size, bool reset, uint8_t *rx_buffer);
void clear_pipeline();
void execute_pipeline();
void agb_write_rom_sequential_pipeline(uint32_t addr, const uint16_t *data, size_t size, bool hwaddr, bool reset);
void agb_write_rom_with_address_pipeline(uint32_t *addrs, uint16_t *datas, size_t size, bool hwaddr);

#ifdef __cplusplus
}
#endif



#ifdef __cplusplus

namespace bacon {
// transfer function ptr type
typedef vecbytes (*transfer_func)(const std::vector<BitArray> &commands);
void ResetChip();
int PowerControl(bool v3_3v, bool v5v);
vecbytes AGBReadROM(uint32_t addr, uint32_t size, bool hwaddr = false, bool reset = true);
void AGBCartWriteROMSequential(uint32_t addr, const std::vector<uint16_t> &data, bool hwaddr = false, bool reset = true, transfer_func transfer = transfer);
void AGBWriteROMWithAddress(const std::vector<std::pair<uint32_t, uint16_t>> &commands, bool hwaddr = false, transfer_func transfer = transfer);
vecbytes AGBReadRAM(uint16_t addr, uint32_t size, bool reset = true);

} // namespace bacon

#endif

#endif // __BACON_H__