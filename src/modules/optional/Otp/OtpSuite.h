#pragma once

#include "OtpLink.h"

namespace otp
{
// Test groups behind the space key. Each press queues one group; poll() hands the cases to the link one at a time
// and judges every result. Only the device whose key was pressed knows about the group, the peer just follows.
//
//   speed   one block per baud step, every step is run even after a failed one
//   faults  generator breaks something on purpose, the case passes when the link notices with the expected reason
//   tuning  round trips and a window sweep at the best clean speed
class Suite
{
  public:
    enum Group : uint8_t { NONE, SPEED, FAULTS, TUNING };

    Suite(Link &link, Logger *log) : link(link), log(log) {}

    bool press(uint32_t now); // false while the demo block is not done or a group is running
    void poll(uint32_t now);

    bool running() const { return running_; }
    Group next() const { return next_; } // what the next press starts
    Group group() const { return group_; }
    static const char *name(Group g);
    unsigned step() const { return idx; }  // cases finished in the running group
    unsigned steps() const { return n; }   // cases in the running group
    const char *current() const { return n && idx < n ? cases[idx].name : ""; }
    const char *last() const { return lastLine; } // short outcome of the newest case for the screen

  private:
    struct Case {
        char name[16];
        Run run;
        char expect[40]; // failure reason substrings separated by '|'; empty: the run must succeed
        bool needPeer;   // cannot fail in loopback (both ends are one UART), skipped there
    };

    void plan(Group g);
    void add(const char *name, const Run &r, const char *expect, bool needPeer = false);
    void judge(const Result &r, uint32_t now);
    void say(const char *fmt, ...) __attribute__((format(printf, 2, 3)));
    static bool matches(const char *reason, const char *expect);

    Link &link;
    Logger *log;
    Group next_ = SPEED;   // what the next press starts
    Group group_ = NONE;   // running group
    bool running_ = false;
    bool waiting = false;  // a case was handed to the link and its result is not in
    Case cases[16];
    unsigned n = 0, idx = 0;
    unsigned seqAtStart = 0;
    uint32_t startedAt = 0, finishedAt = 0, timeoutMs = 0;
    unsigned okCount = 0, failCount = 0, skipCount = 0;
    uint32_t bestBaud = OTP_BAUD; // highest speed whose speed-group block passed
    char lastLine[32] = "";
};
} // namespace otp
