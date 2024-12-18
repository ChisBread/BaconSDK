#ifndef __BACON_H__
#define __BACON_H__
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

void set_dev_path(const char *path);
void set_speed(uint32_t speed);
void spi_init(void);

uint8_t _reverse_bits(uint8_t byte);
uint16_t _reverse_bits_16bit(uint16_t word);
uint8_t reverse_bits(uint8_t byte);
uint16_t reverse_bits_16bit(uint16_t word);

#ifdef __cplusplus
}
#endif

#endif // __BACON_H__