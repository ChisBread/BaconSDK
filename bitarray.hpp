#pragma once

#include <vector>
#include <cstdint>
#include <initializer_list>

namespace bacon {

typedef std::vector<uint8_t> vecbytes;

class BitArray {
private:
    vecbytes data;
    size_t bit_size;
public:
    BitArray() : data(), bit_size(0) {}
    BitArray(const BitArray &other) : data(other.data), bit_size(other.bit_size) {}
    BitArray(BitArray &&other) : data(std::move(other.data)), bit_size(other.bit_size) {}
    BitArray &operator=(const BitArray &other) {
        data = other.data;
        bit_size = other.bit_size;
        return *this;
    }
    BitArray &operator=(BitArray &&other) {
        data = std::move(other.data);
        bit_size = other.bit_size;
        return *this;
    }
    BitArray(vecbytes data) : data(data), bit_size(data.size() * 8) {}
    BitArray(size_t bit_size) : data((bit_size + 7) / 8, 0), bit_size(bit_size) {}
    // 列表初始化{0,1,0,1,1,0,1,0}
    BitArray(std::initializer_list<bool> list) : data((list.size() + 7) / 8, 0), bit_size(list.size()) {
        size_t i = 0;
        for (bool b : list) {
            set(i++, b);
        }
    }
    void reserve(size_t bit_size) {
        data.reserve((bit_size + 7) / 8);
    }
    // 位操作
    bool get(size_t idx) const {
        return (data[idx / 8] >> (7 - idx % 8)) & 1;
    }
    void set(size_t idx, bool value) {
        if (value) {
            data[idx / 8] |= 1 << (7 - idx % 8);
        } else {
            data[idx / 8] &= ~(1 << (7 - idx % 8));
        }
    }
    void push_back(bool value) {
        if (bit_size % 8 == 0) {
            data.push_back(0);
        }
        data.back() |= value << (7 - bit_size % 8);
        bit_size++;
    }
    void push_back(const BitArray &other) {
        size_t old_bytes_len = data.size();
        size_t movbits = bit_size % 8;
        if (movbits == 0) {
            data.insert(data.end(), other.data.begin(), other.data.end());
            bit_size += other.bit_size;
            return;
        }
        data.resize((bit_size + other.bit_size + 7) / 8, 0);
        for (size_t start_byte = old_bytes_len-1; start_byte < data.size(); start_byte++) {
            uint8_t next_other_byte = 0;
            if (start_byte-(old_bytes_len-1) < other.data.size()) {
                next_other_byte = other.data[start_byte - (old_bytes_len - 1)];
            }
            data[start_byte] = data[start_byte] | (next_other_byte >> movbits);
            if (start_byte + 1 < data.size()) {
                data[start_byte + 1] = next_other_byte << (8 - movbits);
            }
        }
        bit_size += other.bit_size;
    }
    size_t size() const {
        return bit_size;
    }
    vecbytes bytes() const {
        return data;
    }
};

} // namespace bacon