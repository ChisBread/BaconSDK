
# Improvement

## 1. Use Google Performance Tools
```bash
sudo apt-get install google-perftools libgoogle-perftools-dev
```

## 2. Use Google TCMalloc With Shared Library
```bash
LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libtcmalloc_minimal.so.4 ./your_program
```

## 3. Use Google TCMalloc With Static Library
```bash
g++ -std=c++20 -O3 bacon.cc -ltcmalloc_minimal
```