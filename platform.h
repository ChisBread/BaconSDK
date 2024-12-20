#ifndef __PLATFORM_H__
#define __PLATFORM_H__
#include <cstdint>
#include <vector>
#include "bitarray.hpp"

#define SPI_BUFFER_SIZE 0x1000

namespace bacon {

    void spi_init(const char *user_path = nullptr, uint32_t user_speed = 0);
    void spi_close();
    void transfer(uint8_t const *tx, uint8_t *rx, size_t len);
    vecbytes transfer(const vecbytes &tx_buffer);
    vecbytes transfer(const std::vector<BitArray> &commands);
    vecbytes transfer(const BitArray &command);

} // namespace bacon

#endif // __PLATFORM_H__