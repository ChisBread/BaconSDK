#ifndef __BACON_H__
#define __BACON_H__

#ifdef __cplusplus
extern "C" {
typedef unsigned char bool;
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
#endif

void set_speed(uint32_t speed);
void spi_init(void);

#ifdef __cplusplus
}
#endif

#endif // __BACON_H__