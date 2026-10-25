#if defined(OTP_APP) || defined(PIO_UNIT_TESTING)

#include "OtpSuite.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace otp
{
const char *Suite::name(Group g)
{
    static const char *const names[] = {"none", "speed", "faults", "tuning"};
    return names[g];
}

void Suite::say(const char *fmt, ...)
{
    if (!log)
        return;
    char s[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s, sizeof(s), fmt, ap);
    va_end(ap);
    log->line(s);
}

void Suite::add(const char *nm, const Run &r, const char *expect, bool needPeer, unsigned observeMs)
{
    if (n >= sizeof(cases) / sizeof(cases[0]))
        return;
    Case &c = cases[n++];
    snprintf(c.name, sizeof(c.name), "%s", nm);
    c.run = r;
    snprintf(c.expect, sizeof(c.expect), "%s", expect);
    c.needPeer = needPeer;
    c.observeMs = observeMs;
}

void Suite::plan(Group g)
{
    n = idx = 0;
    char nm[16];
    Run r;

    if (g == SPEED) {
        // block size grows with the speed, about half a second each (the whole run through all groups should take under 30 s); 115200 quiet is the baseline for the LCD cost
        // against the demo's hexdump
        static const struct {
            uint32_t baud;
            unsigned kib;
        } steps[] = {{115200, 6}, {230400, 12}, {460800, 24}, {921600, 48}, {2000000, 100}, {3000000, 128}, {4000000, 128}, {5000000, 128}};
        for (auto &st : steps) {
            snprintf(nm, sizeof(nm), "%u", (unsigned)st.baud);
            r = Run();
            r.baud = st.baud;
            r.kib = st.kib;
            add(nm, r, "");
        }
    } else if (g == FAULTS) {
        // fresh random block must pass, then every kind of bad data and bad link must be noticed
        r = Run();
        r.kib = 4;
        add("E0 random", r, "");
        static const struct {
            const char *nm;
            Source src;
        } bad[] = {{"E1 text", TEXT}, {"E2 zeros", ZEROS}, {"E3 counter", COUNTER}, {"E4 repeat64", REPEAT64}, {"E5 biased", BIASED}};
        for (auto &e : bad) {
            r = Run();
            r.kib = 4;
            r.src = e.src;
            // two devices: the acceptor trips first and goes silent, so the generator sees a stall; its log has the real reason
            add(e.nm, r, "low entropy|stall");
        }
        static const struct {
            const char *nm;
            Fault f;
            uint32_t baud;
            const char *expect;
        } broken[] = {
            {"L1 flip", FLIP, 115200, "echo"},
            {"L2 drop", DROP, 115200, "echo"},
            {"L3 stall", STALL, 115200, "stall"},
            {"L4 glitch", GLITCH, 115200, "echo|stall|low entropy"},
            {"L7 noswitch", NO_SWITCH, 921600, "echo|stall|low entropy"},
        };
        for (auto &e : broken) {
            r = Run();
            r.kib = 4;
            r.baud = e.baud;
            r.fault = e.f;
            // a speed fault shows only between two UARTs
            add(e.nm, r, e.expect, e.f == GLITCH || e.f == NO_SWITCH);
        }

        // idle link faults: the peer fails and is back within the heartbeat window, so two devices see a gap in its heartbeat
        // (judged after observeMs), a loopback sees its own junk or silence at once
        r = Run();
        r.kib = 1;
        r.fault = STRAY;
        add("L5 stray", r, "junk byte|HB missed|peer silent", false, 1000);
        r.fault = SKIP_HB;
        add("L6 skiphb", r, "HB missed|HB after|peer silent", false, 1900);
    } else if (g == TUNING) {
        // round trips first (window 1), then the window sweep, all at the best clean speed of the speed group
        r = Run();
        r.baud = bestBaud;
        r.kib = 1;
        r.window = 1;
        add("rtt", r, "");
        static const unsigned windows[] = {16, 64, 256, 1024};
        for (unsigned w : windows) {
            snprintf(nm, sizeof(nm), "win %u", w);
            r = Run();
            r.baud = bestBaud;
            r.kib = 8;
            r.window = w;
            add(nm, r, "");
        }
    }
}

bool Suite::press(uint32_t now)
{
    if (running_ || !link.demoDone() || !link.peerAlive(now))
        return false;
    group_ = next_;
    next_ = group_ == SPEED ? FAULTS : group_ == FAULTS ? TUNING : SPEED;
    plan(group_);
    running_ = true;
    waiting = false;
    okCount = failCount = skipCount = 0;
    finishedAt = now;
    say("suite %s start, %u cases", name(group_), n);
    return true;
}

// reason contains one of the substrings in expect, split at '|'
bool Suite::matches(const char *reason, const char *expect)
{
    char buf[40];
    snprintf(buf, sizeof(buf), "%s", expect);
    for (char *p = strtok(buf, "|"); p; p = strtok(nullptr, "|"))
        if (strstr(reason, p))
            return true;
    return false;
}

void Suite::judge(const Result &r, uint32_t now)
{
    Case &c = cases[idx];
    bool pass;
    char got[48];
    if (!c.expect[0]) {
        pass = r.ok;
        if (r.ok)
            snprintf(got, sizeof(got), "ok %u B/s", (unsigned)r.bytesPerSec);
        else
            snprintf(got, sizeof(got), "%s", r.reason);

        // the speed group remembers how fast it went
        if (pass && group_ == SPEED && r.baud > bestBaud)
            bestBaud = r.baud;
    } else {
        pass = !r.ok && matches(r.reason, c.expect);
        snprintf(got, sizeof(got), "%s", r.ok ? "no failure" : r.reason);
    }
    say("case %s expected \"%s\" got \"%s\" %s", c.name, c.expect[0] ? c.expect : "ok", got, pass ? "PASS" : "FAIL");
    snprintf(lastLine, sizeof(lastLine), "%s %s", c.name, pass ? "PASS" : "FAIL");
    pass ? okCount++ : failCount++;
    idx++;
    waiting = false;
    finishedAt = now;
}

void Suite::poll(uint32_t now)
{
    if (!running_)
        return;

    if (idx >= n) {
        say("suite %s done %u ok %u failed %u skipped", name(group_), okCount, failCount, skipCount);
        running_ = false;
        return;
    }

    if (!waiting) {
        // a short settle between cases; a failed case left both ends silent for 250..500 ms and peerAlive waits for the heartbeat
        if (!link.peerAlive(now) || now - finishedAt < 150)
            return;
        Case &c = cases[idx];
        if (c.needPeer && link.loopback()) {
            say("case %s skipped: loopback", c.name);
            skipCount++;
            idx++;
            return;
        }
        if (!link.startRun(c.run, now))
            return;
        seqAtStart = link.result().seq;
        startedAt = now;
        waiting = true;
        return;
    }

    if (link.result().seq != seqAtStart) {
        judge(link.result(), now);
    } else if (cases[idx].observeMs && link.state() == LINKED && now - startedAt >= cases[idx].observeMs) {
        // no failure of our own: did the peer go quiet for longer than a heartbeat cycle ever takes
        Result seen;
        seen.ok = false;
        if (link.maxHbGap() >= 350)
            snprintf(seen.reason, sizeof(seen.reason), "peer silent %u ms", (unsigned)link.maxHbGap());
        else
            snprintf(seen.reason, sizeof(seen.reason), "no peer gap (max %u ms)", (unsigned)link.maxHbGap());
        judge(seen, now);
    } else if (link.state() == LINKED && now - startedAt > IDLE_TIMEOUT_MS) {
        // a block keeps the link out of LINKED until its own stall timeouts end it (hardware: 4 kB/s, not the wire speed),
        // so only an idle link that never produced a result is a hang: stray byte and skipped heartbeat cases, or a lost REQ
        Result none;
        snprintf(none.reason, sizeof(none.reason), "no result");
        none.ok = cases[idx].expect[0] ? true : false; // an expected failure that never came is a failure, a hang too
        judge(none, now);
    }
}
} // namespace otp

#endif
