#pragma once

#include "OtpConfig.h"
#include <stdint.h>

namespace otp
{
// UART receive errors since power-up, as the driver reports them
struct Errs {
    unsigned fifo = 0;   // hardware FIFO overflow: the loop did not read fast enough
    unsigned ring = 0;   // driver ring full: same cause one stage later
    unsigned frame = 0;  // stop bit missing: wrong speed or noise
    unsigned parity = 0; // unused at 8N1, listed for completeness
    unsigned brk = 0;    // line held low
    Errs operator-(const Errs &o) const { return {fifo - o.fifo, ring - o.ring, frame - o.frame, parity - o.parity, brk - o.brk}; }
};

// round trip time of single byte exchanges, microseconds
struct Rtt {
    uint32_t n = 0, sum = 0, min = 0xffffffff, max = 0;
    void add(uint32_t us)
    {
        n++;
        sum += us;
        if (us < min)
            min = us;
        if (us > max)
            max = us;
    }
    uint32_t avg() const { return n ? sum / n : 0; }
};

// outcome of one run, success or failure, read by whoever drives the link (screen, suite)
struct Result {
    unsigned seq = 0; // counts finished runs
    bool ok = false;
    char reason[40] = "";        // failure text, empty on success
    uint32_t baud = OTP_BAUD, kib = 0, window = 0;
    bool gen = false;            // our role
    uint32_t bytes = 0, ms = 0;
    uint32_t bytesPerSec = 0;
    uint32_t gapUs = 0;          // longest time between two polls during the data phase
    unsigned ringHigh = 0;       // most bytes seen waiting in the RX ring
    Errs errs;                   // UART errors during the run
    Rtt rtt;                     // window 1 runs only
};

// one text line to the host log; the module prints it, tests collect it
struct Logger {
    virtual ~Logger() = default;
    virtual void line(const char *s) = 0;
};
} // namespace otp
