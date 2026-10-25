#pragma once

#include <stdint.h>

namespace otp
{
// stream health gate, one feed() per byte, true while the stream still looks random
// cutoffs and false positive rate (0.53% per 512 KiB of xorshift32) measured in design-docs ref/entropy_gate_proto.cpp
class Entropy
{
  public:
    void reset() { *this = Entropy(); }

    bool feed(uint8_t b)
    {
        // zero byte in x where b equals one of the last 4 bytes
        uint32_t x = last ^ b * 0x01010101u;
        coll += ((x - 0x01010101u) & ~x & 0x80808080u) != 0;

        // b - prev == prev - prev2: runs, counters, any constant step
        step += (uint8_t)(b - 2 * (last & 0xff) + (last >> 8 & 0xff)) == 0;

        // guess b from the last 2 bytes through a hashed table (LZP style), then learn it
        uint8_t &guess = table[((last & 0xffff) * 0x9E3779B1u) >> 24];
        hit += guess == b;
        guess = b;

        ones += popcount(b);
        last = last << 8 | b;

        // these counts only grow inside a window, so they trip at once
        if (coll >= 17 || step >= 9 || hit >= 10)
            return false;
        if (++n < 256)
            return true;

        // window end: bit balance must sit within 4.9 sd of 1024
        bool balanced = ones > 1024 - 110 && ones < 1024 + 110;
        n = coll = step = hit = ones = 0;
        return balanced;
    }

  private:
    // xtensa has no popcount instruction, __builtin_popcount would call into libgcc
    static unsigned popcount(unsigned v)
    {
        v -= (v >> 1) & 0x55;
        v = (v & 0x33) + ((v >> 2) & 0x33);
        return (v + (v >> 4)) & 0x0f;
    }

    uint32_t last = 0;    // last 4 bytes, newest in the low byte
    unsigned n = 0;       // bytes in this window
    unsigned coll = 0;    // b matched one of the last 4 bytes
    unsigned step = 0;    // second difference was zero
    unsigned hit = 0;     // table guessed b
    unsigned ones = 0;    // set bits in this window
    uint8_t table[256] = {};
};
} // namespace otp
