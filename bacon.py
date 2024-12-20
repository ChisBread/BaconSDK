#coding:utf8

#libbacon.so binding

# int spi_init(const char *user_path, uint32_t user_speed, bool verbose);
# void spi_close();
# void reset_chip();
# int power_control(bool v3_3v, bool v5v);
# void agb_read_rom(uint32_t addr, uint32_t size, bool hwaddr, bool reset, uint8_t *rx_buffer);
# void agb_write_rom_sequential(uint32_t addr, const uint16_t *data, size_t size, bool hwaddr, bool reset);
# void agb_write_rom_with_address(const std::pair<uint32_t, uint16_t> *commands, size_t size, bool hwaddr);
# void agb_read_ram(uint16_t addr, uint32_t size, bool reset, uint8_t *rx_buffer);

import ctypes
import os
import sys
import time

ROM_MAX_SIZE = 0x2000000
RAM_MAX_SIZE = 0x20000
RAM_512_SIZE = 0x10000
class Bacon:
    def __init__(self, libbacon=None):
        if libbacon is None:
            find_list = [os.path.join(os.path.dirname(__file__), "build/libbacon.so"), "/usr/local/lib/libbacon.so", "/usr/lib/libbacon.so"]
            for libbacon_path in find_list:
                if os.path.exists(libbacon_path):
                    self.libbacon = ctypes.CDLL(libbacon_path)
                    break
            else:
                print("libbacon.so not found.")
                sys.exit(1)
        else:
            self.libbacon = ctypes.CDLL(libbacon)
        self.libbacon.spi_init.argtypes = [ctypes.c_char_p, ctypes.c_uint32, ctypes.c_ubyte]
        self.libbacon.spi_init.restype = ctypes.c_int
        self.libbacon.spi_close.argtypes = []
        self.libbacon.spi_close.restype = None
        self.libbacon.reset_chip.argtypes = []
        self.libbacon.reset_chip.restype = None
        self.libbacon.power_control.argtypes = [ctypes.c_bool, ctypes.c_bool]
        self.libbacon.power_control.restype = ctypes.c_int
        self.libbacon.agb_read_rom.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_bool, ctypes.c_bool, ctypes.POINTER(ctypes.c_uint8)]
        self.libbacon.agb_read_rom.restype = None
        self.libbacon.agb_write_rom_sequential.argtypes = [ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint16), ctypes.c_size_t, ctypes.c_bool, ctypes.c_bool]
        self.libbacon.agb_write_rom_sequential.restype = None
        self.libbacon.agb_write_rom_with_address.argtypes = [ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint16), ctypes.c_size_t, ctypes.c_bool]
        self.libbacon.agb_write_rom_with_address.restype = None
        self.libbacon.agb_read_ram.argtypes = [ctypes.c_uint16, ctypes.c_uint32, ctypes.c_bool, ctypes.POINTER(ctypes.c_uint8)]
        self.libbacon.agb_read_ram.restype = None

    def spi_init(self, user_path, user_speed, verbose):
        return self.libbacon.spi_init(user_path, user_speed, verbose)
    
    def spi_close(self):
        return self.libbacon.spi_close()

    def reset_chip(self):
        return self.libbacon.reset_chip()

    def power_control(self, v3_3v, v5v):
        return self.libbacon.power_control(v3_3v, v5v)

    def agb_read_rom(self, addr, size, hwaddr = False, reset = True):
        rx_buffer = (ctypes.c_uint8 * size)()
        self.libbacon.agb_read_rom(addr, size, hwaddr, reset, rx_buffer)
        return rx_buffer

    def agb_write_rom_sequential(self, addr, data, size, hwaddr = False, reset = True):
        data = (ctypes.c_uint16 * size)(*data)
        self.libbacon.agb_write_rom_sequential(addr, data, size, hwaddr, reset)

    def agb_write_rom_with_address(self, commands, size, hwaddr = False):
        commands = (
            (ctypes.c_uint32 * size)(*[command[0] for command in commands]),
            (ctypes.c_uint16 * size)(*[command[1] for command in commands])
        )
        self.libbacon.agb_write_rom_with_address(commands[0], commands[1], size, hwaddr)

    def agb_read_ram(self, addr, size, reset = True):
        rx_buffer = (ctypes.c_uint8 * size)()
        self.libbacon.agb_read_ram(addr, size, reset, rx_buffer)
        return rx_buffer

def test_readrom(bacon):
    readsize = ROM_MAX_SIZE
    start = time.time() * 1000
    rom = bacon.agb_read_rom(0, readsize)
    for i in range(10):
        print("%02x" % rom[i], end="")
    print()
    cost = time.time() * 1000 - start
    print("Read ROM data cost %f seconds, speed: %fKB/s" % (cost/1000.0, readsize / (cost/1000.0) / 1024.0))
def test_readram(bacon):
    readsize = RAM_512_SIZE
    start = time.time() * 1000
    ram = bacon.agb_read_ram(0, readsize)
    for i in range(10):
        print("%02x" % ram[i], end="")
    print()
    cost = time.time() * 1000 - start
    print("Read RAM data cost %f seconds, speed: %fKB/s" % (cost/1000.0, readsize / (cost/1000.0) / 1024.0))

if __name__ == "__main__":
    bacon = Bacon()
    bacon.spi_init(b"/dev/spidev3.0", 32000000, True)
    bacon.power_control(True, False) # 3.3V
    
    if sys.argv[1] == "test_readrom":
        test_readrom(bacon)
    elif sys.argv[1] == "test_readram":
        test_readram(bacon)

    bacon.power_control(False, False) # 0V
    bacon.spi_close()
