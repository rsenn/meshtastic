#pragma once

#include "OtpConfig.h"
#include "OtpEntropy.h"
#include <stddef.h>
#include <stdint.h>

// wire format, states and timings: design-docs SERIAL_PROTOCOL.md
namespace otp
{
struct Port {
    virtual ~Port() = default;
    virtual int read() = 0;     // -1 when empty
    virtual int writable() = 0; // free TX bytes
    virtual void write(uint8_t b) = 0;
    virtual void setBaud(uint32_t baud) {}
    virtual void flush() {} // returns when the last byte has left the wire
};

// Fixed rows, the newest line overwrites the oldest: nothing scrolls, the screen just redraws all rows.
struct Rows {
    static constexpr int N = 8;
    static constexpr int W = 40;
    char text[N][W] = {};
    uint8_t cur = 0; // row written last
    uint32_t rev = 0;
    void put(const char *fmt, ...) __attribute__((format(printf, 2, 3)));
    void clear();
};

// what one block transfer looks like, chosen by the generator and sent to the acceptor in SPD
struct Run {
    uint32_t baud = OTP_BAUD;   // speed of the data phase, the acceptor may grant less
    unsigned kib = BLOCK_KIB;   // block length
    unsigned window = WINDOW;   // unechoed bytes in flight, generator only
    bool dump = false;          // hexdump rows, both ends, only sensible at 115200
};

enum State : uint8_t {
    SLEEP,  // silent, drain RX
    BEAT,   // heartbeat, wait for the peer's
    REQ,    // we asked to generate, wait for ACC
    NEGO,   // ACC done, SPD asked (generator) or awaited (acceptor)
    SWITCH, // SPD answered, silent while both ends change speed
    GEN,    // generator: send the block, check every echoed byte
    ACCEPT, // acceptor: echo every byte
    LINKED  // idle, heartbeat only
};

class Link
{
  public:
    Link(Port &port, uint32_t seed) : port(port), rng(seed ? seed : 1) {}

    void start(uint32_t now);
    void poll(uint32_t now);
    bool startRun(const Run &r, uint32_t now); // LINKED only: new block
    void restart(uint32_t now) { startRun(Run(), now); }

    State state() const { return state_; }
    bool active(uint32_t now) const { return everRx && (uint32_t)(now - lastRx) < ACTIVE_MS; }
    uint32_t done() const { return doneCnt; }
    uint32_t total() const { return blockBytes; }
    const Run &run() const { return run_; }
    uint32_t baud() const { return curBaud; }
    bool demoDone() const { return demoDone_; }
    uint32_t revision() const { return rx.rev + tx.rev + failures; }

    Rows rx, tx;
    uint32_t failures = 0;

  private:
    void onByte(uint8_t c, uint32_t now);
    void onFrame(uint8_t type, uint32_t value, uint32_t now);
    void onSpd(uint32_t now);
    void bad(const char *what, uint32_t now);
    bool send(const uint8_t *p, size_t n, uint32_t now);
    bool sendNonce(uint8_t type, uint32_t nonce, uint32_t now);
    bool sendSpd(uint32_t baud, unsigned kib, bool dump, uint32_t now);
    void sendReq(uint32_t now);
    void beginSwitch(State next, uint32_t now);
    void beginData(State s, uint32_t now);
    void pumpGen(uint32_t now);
    void pumpAccept(uint32_t now);
    void finish(uint32_t now);
    void fail(uint32_t now, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
    void setBaud(uint32_t baud);
    bool framing() const { return state_ == BEAT || state_ == REQ || state_ == NEGO || state_ == LINKED; }
    void hexLine(Rows &r, const char *tag, const uint8_t *ring, uint32_t n);
    uint32_t random();

    Port &port;
    uint32_t rng; // xorshift32 state for nonces and the silence length
    State state_ = SLEEP;
    State switchTo = GEN;                // state after SWITCH
    bool gen = false;                    // our role in the current exchange
    bool demoDone_ = false;              // first block completed, later relinks need no block
    Run want;                            // what we ask for in the next REQ
    Run run_;                            // what is running now
    uint32_t curBaud = OTP_BAUD;         // speed the UART is set to
    uint32_t blockBytes = BLOCK_BYTES;   // length of the current block
    uint32_t sleepAt = 0, sleepFor = 0;  // silence after a failure
    uint32_t lastBeat = 0;               // our last heartbeat sent
    uint32_t lastHb = 0;                 // peer heartbeat last seen
    uint32_t enteredAt = 0;              // LINKED entered
    uint32_t reqAt = 0;                  // REQ sent
    uint32_t negoAt = 0, switchAt = 0;   // NEGO and SWITCH entered
    uint32_t frameAt = 0;                // first byte of the open frame
    uint32_t lastRx = 0;                 // any byte received, for "Activity!"
    uint32_t lastLog = 0;                // hexdump and heartbeat log throttles
    uint32_t lastProgress = 0;           // last byte of the data phase
    bool everRx = false, hbValid = false;
    uint8_t buf[10];                     // frame being received
    uint8_t pos = 0;                     // bytes in buf, 0 = no open frame
    uint32_t myNonce = 0;                // nonce of our REQ, or the one we accepted
    uint32_t sendRng = 0, checkRng = 0;  // block generator and its mirror for the echo check
    uint32_t sentCnt = 0, doneCnt = 0;   // bytes sent, bytes echoed and verified
    Entropy gate;                        // every block byte passes it, on both ends
    uint8_t txRing[6] = {}, rxRing[6] = {}; // last bytes for the hexdump rows
    uint32_t txN = 0, rxN = 0;
};
} // namespace otp
