#pragma once

#include "OtpConfig.h"
#include "OtpEntropy.h"
#include <stddef.h>
#include <stdint.h>

namespace otp
{
struct Port {
    virtual ~Port() = default;
    virtual int read() = 0;     // -1 when empty
    virtual int writable() = 0; // free TX bytes
    virtual void write(uint8_t b) = 0;
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

enum State : uint8_t {
    SLEEP,  // silent, drain RX
    BEAT,   // heartbeat, wait for the peer's
    REQ,    // we asked to generate, wait for ACC
    GEN,    // generator: send the block, check every echoed byte
    ACCEPT, // acceptor: echo every byte
    LINKED  // block exchanged, heartbeat only
};

class Link
{
  public:
    Link(Port &port, uint32_t seed) : port(port), rng(seed ? seed : 1) {}

    void start(uint32_t now);
    void poll(uint32_t now);
    void restart(uint32_t now); // LINKED only: new handshake

    State state() const { return state_; }
    bool active(uint32_t now) const { return everRx && (uint32_t)(now - lastRx) < ACTIVE_MS; }
    uint32_t done() const { return doneCnt; }
    uint32_t revision() const { return rx.rev + tx.rev + failures; }

    Rows rx, tx;
    uint32_t failures = 0;

  private:
    void onByte(uint8_t c, uint32_t now);
    void onFrame(uint8_t type, uint32_t nonce, uint32_t now);
    void bad(const char *what, uint32_t now);
    bool send(const uint8_t *p, size_t n, uint32_t now);
    bool sendNonce(uint8_t type, uint32_t nonce, uint32_t now);
    void sendReq(uint32_t now);
    void beginData(State s, uint32_t now);
    void pumpGen(uint32_t now);
    void pumpAccept(uint32_t now);
    void finish(uint32_t now);
    void fail(uint32_t now, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
    void hexLine(Rows &r, const char *tag, const uint8_t *ring, uint32_t n);
    uint32_t random();

    Port &port;
    uint32_t rng;
    State state_ = SLEEP;
    uint32_t sleepAt = 0, sleepFor = 0;
    uint32_t lastBeat = 0, lastHb = 0, enteredAt = 0, reqAt = 0, frameAt = 0, lastRx = 0, lastLog = 0, lastProgress = 0;
    bool everRx = false, hbValid = false;
    uint8_t buf[8];
    uint8_t pos = 0;
    uint32_t myNonce = 0, sendRng = 0, checkRng = 0, sentCnt = 0, doneCnt = 0;
    Entropy gate; // every block byte passes it, on both ends
    uint8_t txRing[6] = {}, rxRing[6] = {};
    uint32_t txN = 0, rxN = 0;
};
} // namespace otp
