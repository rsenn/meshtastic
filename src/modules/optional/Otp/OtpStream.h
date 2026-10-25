#pragma once

#include <stdint.h>

namespace otp
{
// xorshift32: stand-in for real entropy, one byte per step
inline uint8_t xorshift8(uint32_t &x)
{
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return (uint8_t)(x >> 8);
}

// what the generator puts in the block; everything but RANDOM must make the entropy gate trip
enum Source : uint8_t { RANDOM, TEXT, ZEROS, COUNTER, REPEAT64, BIASED };

// block byte generator, one next() per byte; the echo check runs a second copy from the same seed
class Stream
{
  public:
    Stream() {}
    Stream(Source kind, uint32_t seed) : kind(kind), x(seed ? seed : 1)
    {
        if (kind == REPEAT64)
            for (auto &b : pat)
                b = xorshift8(x);
    }

    uint8_t next()
    {
        switch (kind) {
        case TEXT:
            return nextText();
        case ZEROS:
            return 0;
        case COUNTER:
            return (uint8_t)i++;
        case REPEAT64:
            return pat[i++ % 64];
        case BIASED:
            return xorshift8(x) & xorshift8(x); // 25% of the bits set
        default:
            return xorshift8(x);
        }
    }

  private:
    // random words from a small list, so it looks like text but never cycles
    uint8_t nextText()
    {
        static const char *const words[8] = {"the", "link", "pad", "key", "byte", "secret", "and", "is"};
        const char *w = words[word];
        if (i < 6 && w[i]) // the longest word has 6 letters
            return (uint8_t)w[i++];
        i = 0;
        word = xorshift8(x) & 7;
        return ' ';
    }

    Source kind = RANDOM;
    uint32_t x = 1;     // xorshift32 state
    unsigned i = 0;     // position: counter value, pattern index or letter in the word
    unsigned word = 0;  // TEXT: index of the current word
    uint8_t pat[64] = {}; // REPEAT64: the pattern
};
} // namespace otp
