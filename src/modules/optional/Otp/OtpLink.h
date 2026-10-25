#pragma once

#include "OtpConfig.h"
#include "OtpEntropy.h"
#include "OtpStats.h"
#include "OtpStream.h"
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
    virtual void flush() {}                   // returns when the last byte has left the wire
    virtual int available() { return 0; }     // bytes waiting in the RX ring
    virtual uint32_t us() { return 0; }       // microsecond clock, wraps
    virtual Errs errors() { return Errs(); }  // receive errors since power-up
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

// something the generator breaks on purpose, to see the link notice it
enum Fault : uint8_t {
    NO_FAULT,
    FLIP,      // one block byte goes out with a bit flipped
    DROP,      // one block byte is not sent
    STALL,     // block bytes stop
    GLITCH,    // line speed drops to 9600 for 100 ms mid-block
    NO_SWITCH, // generator never changes speed after SPD
    STRAY,     // LINKED: one stray byte on the line
    SKIP_HB    // LINKED: one heartbeat not sent
};

// what one block transfer looks like, chosen by the generator and sent to the acceptor in SPD
struct Run {
    uint32_t baud = OTP_BAUD;   // speed of the data phase, the acceptor may grant less
    unsigned kib = BLOCK_KIB;   // block length
    unsigned window = WINDOW;   // unechoed bytes in flight, generator only
    bool dump = false;          // hexdump rows, both ends, only sensible at 115200
    Source src = RANDOM;        // block content, generator only
    Fault fault = NO_FAULT;     // generator only, never sent to the peer
    unsigned faultAt = 8192;    // block byte index where the fault hits
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
    bool loopback() const { return loop_; } // the last handshake was with ourselves: speed faults cannot show
    // idle for a second with the peer's heartbeat flowing: the peer has seen ours too, junk would not be ignored as boot noise
    bool peerAlive(uint32_t now) const { return state_ == LINKED && hbValid && now - enteredAt >= 1000; }
    const Result &result() const { return result_; }
    void setLogger(Logger *l) { logger = l; }
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
    void note(const char *fmt, ...) __attribute__((format(printf, 2, 3)));
    void endRun(bool ok, const char *reason, uint32_t now);
    void sampleLoop();
    bool inject(uint8_t &b, uint32_t now);
    bool framing() const { return state_ == BEAT || state_ == REQ || state_ == NEGO || state_ == LINKED; }
    void hexLine(Rows &r, const char *tag, const uint8_t *ring, uint32_t n);
    uint32_t random();

    Port &port;
    uint32_t rng; // xorshift32 state for nonces and the silence length
    State state_ = SLEEP;
    State switchTo = GEN;                // state after SWITCH
    bool gen = false;                    // our role in the current exchange
    bool loop_ = false;                  // our own REQ came back
    bool demoDone_ = false;              // first block completed, later relinks need no block
    Run want;                            // what we ask for in the next REQ
    Run run_;                            // what is running now
    uint32_t curBaud = OTP_BAUD;         // speed the UART is set to
    uint32_t blockBytes = BLOCK_KIB * 1024u;   // length of the current block
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
    bool opened = false;                 // start() ran once
    bool skipHb = false;                 // fault SKIP_HB armed
    bool stalled = false;                // fault STALL fired
    bool glitching = false;              // fault GLITCH is running
    uint32_t glitchAt = 0;
    uint8_t buf[10];                     // frame being received
    uint8_t pos = 0;                     // bytes in buf, 0 = no open frame
    uint32_t myNonce = 0;                // nonce of our REQ, or the one we accepted
    Stream sendS, checkS;                // block bytes and its mirror for the echo check
    uint32_t sentCnt = 0, doneCnt = 0;   // bytes sent, bytes echoed and verified
    Entropy gate;                        // every block byte passes it, on both ends
    Logger *logger = nullptr;            // host log, may be null
    Result result_;                      // newest finished run
    Rtt rtt;                             // round trips of the current run
    Errs errsAtStart;                    // UART errors when the data phase began
    uint32_t runStart = 0;               // data phase began, ms
    uint32_t lastPollUs = 0, gapUs = 0;  // loop gap tracking during the data phase
    uint32_t sendUs = 0;                 // when the single byte in flight left (window 1)
    unsigned ringHigh = 0;               // RX ring fill high water of the run
    uint32_t linkOpenAt = 0;             // first start(), for the peer detected message
    uint8_t txRing[6] = {}, rxRing[6] = {}; // last bytes for the hexdump rows
    uint32_t txN = 0, rxN = 0;
};
} // namespace otp
