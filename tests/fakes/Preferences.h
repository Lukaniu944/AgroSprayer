#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>
class Preferences {
public:
    inline static std::vector<uint8_t> storage{};
    inline static bool failBegin = false;
    inline static bool failWrite = false;
    bool begin(const char*, bool) { return !failBegin; }
    std::size_t putBytes(const char*, const void* data, std::size_t len) {
        if (failWrite) return 0;
        const auto* p = static_cast<const uint8_t*>(data);
        storage.assign(p, p + len);
        return len;
    }
    std::size_t getBytes(const char*, void* data, std::size_t len) {
        const std::size_t n = std::min(len, storage.size());
        if (n) std::memcpy(data, storage.data(), n);
        return n;
    }
    void end() {}
};
