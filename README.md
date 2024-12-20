# Bacon SDK

- Bacon烧录器的SDK，提供了烧录器的基本功能接口
- Bacon SDK is the SDK of Bacon Flasher, which provides basic functional interfaces of the flasher

## Build

```bash
mkdir build
cd build
cmake ..
make -j4
# test
sudo ./bacon_cli test_readrom
```

## Optimization & Usage

## 1. Use Google TCMalloc With Shared Library
```bash
sudo apt-get install google-perftools libgoogle-perftools-dev
sudo LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libtcmalloc_minimal.so ./bacon_cli test_readrom
```