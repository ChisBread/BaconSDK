#ifndef __BACON_H__
#define __BACON_H__
#include <cstdint>

#include "common.h"
#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif
void ResetChip();


#ifdef __cplusplus
}
#endif



#ifdef __cplusplus
namespace bacon {
void ResetChip();
vecbytes AGBReadROM(uint32_t addr, uint32_t size, bool hwaddr = false, bool reset = true);
void AGBCartWriteROMSequential(uint32_t addr, const std::vector<uint16_t> &data, bool hwaddr = false, bool reset = true);
void AGBWriteROMWithAddress(const std::vector<std::pair<uint32_t, uint16_t>> &commands, bool hwaddr = false);
vecbytes AGBReadRAM(uint16_t addr, uint32_t size, bool reset = true);
#endif

} // namespace bacon

#endif // __BACON_H__