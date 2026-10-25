#include "TestUtil.h"
#include "modules/optional/Otp/OtpEntropy.h"
#include "modules/optional/Otp/OtpLink.h"
#include "modules/optional/Otp/OtpSuite.h"
#include <string.h>
#include <deque>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unity.h>

// Serial link state machine of the OTP demo screen: heartbeat, handshake with role pick, 512k echo
// exchange, and bail-out (silence, then relink) on any glitch. The wire is simulated, one tick = 1 ms.
// Regression guarded: a failure must leave both ends silent and able to link again.

using namespace otp;

struct Lines : Logger {
    std::string all;
    void line(const char *s) override
    {
        all += s;
        all += '\n';
    }
    bool has(const char *s) const { return all.find(s) != std::string::npos; }
};

static uint32_t now = 1000;

struct Wire {
    std::deque<uint8_t> q;
    std::deque<uint32_t> bq; // speed the writer was at, one per byte written through a port
    std::deque<uint64_t> rq; // microsecond at which the byte has crossed the wire, one per byte written through a port
    uint64_t freeAtUs = 0;   // wire busy until
    bool cut = false;        // drop everything written
    int corruptAt = -1;      // flip the byte with this running index
    int count = 0;
};

// a byte read at another speed than it was written at arrives as garbage, like on a real UART
struct FakePort : Port {
    Wire *in, *out;
    uint32_t baud = OTP_BAUD, peak = OTP_BAUD;
    bool stuck = false; // ignores setBaud: a peer that never switched
    FakePort(Wire *in, Wire *out) : in(in), out(out) {}
    int read() override
    {
        if (in->q.empty())
            return -1;

        // a byte is readable only once it has crossed the wire: 10 bits at the writer's speed
        if (!in->rq.empty() && in->rq.front() > (uint64_t)now * 1000)
            return -1;
        if (!in->rq.empty())
            in->rq.pop_front();
        int c = in->q.front();
        in->q.pop_front();
        if (!in->bq.empty()) {
            uint32_t wb = in->bq.front();
            in->bq.pop_front();
            if (wb != baud)
                c = (c * 7 + 13) & 0xff; // scrambled, and scrambling twice does not restore it
        }
        return c;
    }
    int writable() override { return 1000; }
    uint32_t us() override { return now * 1000; }
    int available() override { return (int)in->q.size(); }
    void write(uint8_t b) override
    {
        if (out->count++ == out->corruptAt)
            b ^= 0x40;
        if (!out->cut) {
            out->q.push_back(b);
            out->bq.push_back(baud);
            if (out->freeAtUs < (uint64_t)now * 1000)
                out->freeAtUs = (uint64_t)now * 1000;
            out->freeAtUs += 10000000ull / baud;
            out->rq.push_back(out->freeAtUs);
        }
    }
    void setBaud(uint32_t b) override
    {
        if (stuck)
            return;
        baud = b;
        if (b > peak)
            peak = b;
    }
};


static bool runUntil(Link &a, Link *b, State want, uint32_t maxMs)
{
    for (uint32_t i = 0; i < maxMs; i++, now++) {
        a.poll(now);
        if (b)
            b->poll(now);
        if (a.state() == want && (!b || b->state() == want))
            return true;
    }
    return false;
}

static void startBoth(Link &a, Link &b)
{
    a.start(now);
    b.start(now);
}

static void test_loopback_reaches_linked()
{
    Wire w;
    FakePort p(&w, &w);
    Link a(p, 1234);
    a.start(now);
    TEST_ASSERT_TRUE(runUntil(a, nullptr, LINKED, 10000));
    TEST_ASSERT_EQUAL_UINT32(BLOCK_KIB * 1024u, a.done());
    TEST_ASSERT_EQUAL_UINT32(0, a.failures);
}

static void test_two_peers_pick_roles_and_link()
{
    Wire ab, ba;
    FakePort pa(&ba, &ab), pb(&ab, &ba);
    Link a(pa, 11), b(pb, 22);
    startBoth(a, b);
    bool sawGen = false, sawAccept = false;
    for (uint32_t i = 0; i < 10000 && !(a.state() == LINKED && b.state() == LINKED); i++, now++) {
        a.poll(now);
        b.poll(now);
        sawGen |= a.state() == GEN || b.state() == GEN;
        sawAccept |= a.state() == ACCEPT || b.state() == ACCEPT;
    }
    TEST_ASSERT_TRUE(sawGen && sawAccept);
    TEST_ASSERT_EQUAL(LINKED, a.state());
    TEST_ASSERT_EQUAL(LINKED, b.state());
    TEST_ASSERT_EQUAL_UINT32(0, a.failures + b.failures);
}

static void test_corrupted_byte_in_transfer_fails_both_then_relinks()
{
    Wire ab, ba;
    FakePort pa(&ba, &ab), pb(&ab, &ba);
    Link a(pa, 33), b(pb, 44);
    startBoth(a, b);
    for (uint32_t i = 0; i < 5000 && a.state() != GEN && b.state() != GEN; i++, now++) {
        a.poll(now);
        b.poll(now);
    }
    Wire &toAcceptor = (a.state() == GEN) ? ab : ba;
    toAcceptor.corruptAt = toAcceptor.count + 1000;
    for (uint32_t i = 0; i < 3000 && (a.failures + b.failures) < 2; i++, now++) {
        a.poll(now);
        b.poll(now);
    }
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(1, a.failures);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(1, b.failures);
    TEST_ASSERT_TRUE(runUntil(a, &b, LINKED, 20000));
}

static void test_cut_wire_in_transfer_times_out_both()
{
    Wire ab, ba;
    FakePort pa(&ba, &ab), pb(&ab, &ba);
    Link a(pa, 55), b(pb, 66);
    startBoth(a, b);
    for (uint32_t i = 0; i < 5000 && a.state() != GEN && b.state() != GEN; i++, now++) {
        a.poll(now);
        b.poll(now);
    }
    ab.cut = ba.cut = true;
    for (uint32_t i = 0; i < 1000; i++, now++) {
        a.poll(now);
        b.poll(now);
    }
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(1, a.failures);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(1, b.failures);
    ab.cut = ba.cut = false;
    TEST_ASSERT_TRUE(runUntil(a, &b, LINKED, 20000));
}

static void test_missed_heartbeat_when_linked_resets_both()
{
    Wire ab, ba;
    FakePort pa(&ba, &ab), pb(&ab, &ba);
    Link a(pa, 77), b(pb, 88);
    startBoth(a, b);
    TEST_ASSERT_TRUE(runUntil(a, &b, LINKED, 10000));
    for (uint32_t i = 0; i < 1000; i++, now++) {
        a.poll(now);
        b.poll(now);
    }
    ba.cut = true; // b goes mute
    for (uint32_t i = 0; i < 1200; i++, now++) {
        a.poll(now);
        b.poll(now);
    }
    TEST_ASSERT_EQUAL(SLEEP, a.state());
    ba.cut = false;
    TEST_ASSERT_TRUE(runUntil(a, &b, LINKED, 20000));
}

static void test_junk_byte_when_linked_resets()
{
    Wire ab, ba;
    FakePort pa(&ba, &ab), pb(&ab, &ba);
    Link a(pa, 99), b(pb, 100);
    startBoth(a, b);
    TEST_ASSERT_TRUE(runUntil(a, &b, LINKED, 10000));
    ba.q.push_back(0x55);
    a.poll(now++);
    TEST_ASSERT_EQUAL(SLEEP, a.state());
    TEST_ASSERT_TRUE(runUntil(a, &b, LINKED, 20000));
}

static void test_key_starts_new_handshake_without_failure()
{
    Wire ab, ba;
    FakePort pa(&ba, &ab), pb(&ab, &ba);
    Link a(pa, 5), b(pb, 6);
    startBoth(a, b);
    TEST_ASSERT_TRUE(runUntil(a, &b, LINKED, 10000));
    a.restart(now);
    TEST_ASSERT_EQUAL(REQ, a.state());
    for (uint32_t i = 0; i < 200 && a.state() != GEN; i++, now++) {
        a.poll(now);
        b.poll(now);
    }
    TEST_ASSERT_EQUAL(GEN, a.state());
    TEST_ASSERT_TRUE(runUntil(a, &b, LINKED, 10000));
    TEST_ASSERT_EQUAL_UINT32(0, a.failures + b.failures);
}

static void test_simultaneous_requests_resolve_to_one_generator()
{
    Wire ab, ba;
    FakePort pa(&ba, &ab), pb(&ab, &ba);
    Link a(pa, 7), b(pb, 8);
    startBoth(a, b);
    TEST_ASSERT_TRUE(runUntil(a, &b, LINKED, 10000));
    a.restart(now);
    b.restart(now);
    TEST_ASSERT_TRUE(runUntil(a, &b, LINKED, 10000));
    TEST_ASSERT_EQUAL_UINT32(0, a.failures + b.failures);
}


// entropy gate: xorshift32 blocks stay below 2% trips over 512 KiB (measured 0.53%), structured streams trip early
static uint8_t xorshift(uint32_t &x)
{
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return (uint8_t)(x >> 8);
}

static const uint32_t GATE_BLOCK = 512u * 1024u; // the false positive rate is quoted per 512 KiB

static void test_entropy_gate_passes_xorshift_blocks()
{
    int trips = 0;
    uint32_t seed = 12345;
    for (int run = 0; run < 400; run++) {
        uint32_t x = (seed = seed * 1664525u + 1013904223u) ^ ((uint32_t)run * 2654435761u);
        Entropy e;
        for (uint32_t i = 0; i < GATE_BLOCK; i++) {
            if (!e.feed(xorshift(x))) {
                trips++;
                break;
            }
        }
    }
    TEST_ASSERT_LESS_OR_EQUAL_INT(8, trips);
}

static unsigned tripsAt(uint8_t (*gen)(unsigned i))
{
    Entropy e;
    for (unsigned i = 0; i < 4096; i++)
        if (!e.feed(gen(i)))
            return i;
    return 0;
}

static uint8_t genText(unsigned i)
{
    static const char s[] = "all work and no play makes jack a dull boy, the quick brown fox jumps over the lazy dog ";
    return (uint8_t)s[i % (sizeof(s) - 1)];
}
static uint8_t genZero(unsigned) { return 0; }
static uint8_t genCounter(unsigned i) { return (uint8_t)i; }
static uint8_t genStep3(unsigned i) { return (uint8_t)(i * 3 + 7); }
static uint8_t genRepeat64(unsigned i)
{
    static uint8_t pat[64];
    static bool init = false;
    if (!init) {
        uint32_t x = 99;
        for (auto &b : pat)
            b = xorshift(x);
        init = true;
    }
    return pat[i % 64];
}
static uint8_t genBiased(unsigned)
{
    static uint32_t x = 7;
    return xorshift(x) & xorshift(x);
}

static void test_entropy_gate_trips_on_structure()
{
    TEST_ASSERT_GREATER_THAN_UINT32(0, tripsAt(genText));
    TEST_ASSERT_GREATER_THAN_UINT32(0, tripsAt(genZero));
    TEST_ASSERT_GREATER_THAN_UINT32(0, tripsAt(genCounter));
    TEST_ASSERT_GREATER_THAN_UINT32(0, tripsAt(genStep3));
    TEST_ASSERT_GREATER_THAN_UINT32(0, tripsAt(genRepeat64));
    TEST_ASSERT_GREATER_THAN_UINT32(0, tripsAt(genBiased));
}

static uint8_t crc8Ref(const uint8_t *p, size_t n)
{
    uint8_t c = 0;
    while (n--) {
        c ^= *p++;
        for (int i = 0; i < 8; i++)
            c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
    }
    return c;
}

static void test_acceptor_rejects_structured_block()
{
    Wire in, out;
    FakePort p(&in, &out);
    Link b(p, 3);
    b.start(now);
    // peer asks to generate (REQ), we accept, then it streams zeros
    uint8_t req[7] = {SYNC, T_REQ, 1, 2, 3, 4, 0};
    req[6] = crc8Ref(req + 1, 5);
    for (uint8_t v : req)
        in.q.push_back(v);
    // then its SPD: 115200, 16k, no dump
    uint8_t spd[10] = {SYNC, T_SPD, 0x00, 0xC2, 0x01, 0x00, 0x10, 0x00, 0, 0}; // 16 KiB
    spd[9] = crc8Ref(spd + 1, 8);
    for (int i = 0; i < 5 && b.state() != NEGO; i++, now++)
        b.poll(now);
    TEST_ASSERT_EQUAL(NEGO, b.state());
    for (uint8_t v : spd)
        in.q.push_back(v);
    for (int i = 0; i < 100 && b.state() != ACCEPT; i++, now++)
        b.poll(now);
    TEST_ASSERT_EQUAL(ACCEPT, b.state());
    for (int i = 0; i < 2000; i++)
        in.q.push_back(0);
    for (int i = 0; i < 20 && b.state() == ACCEPT; i++, now++)
        b.poll(now);
    TEST_ASSERT_EQUAL(SLEEP, b.state());
    TEST_ASSERT_EQUAL_UINT32(1, b.failures);
    TEST_ASSERT_NOT_NULL(strstr(b.rx.text[b.rx.cur], "low entropy"));
}

static void test_speed_negotiated_block_runs_fast_and_returns_to_base()
{
    Wire ab, ba;
    FakePort pa(&ba, &ab), pb(&ab, &ba);
    Link a(pa, 31), b(pb, 32);
    startBoth(a, b);
    TEST_ASSERT_TRUE(runUntil(a, &b, LINKED, 10000));
    Run r;
    r.baud = 921600;
    r.kib = 64;
    TEST_ASSERT_TRUE(a.startRun(r, now));
    bool fast = false;
    for (uint32_t i = 0; i < 5000 && !(i > 50 && a.state() == LINKED && b.state() == LINKED); i++, now++) {
        a.poll(now);
        b.poll(now);
        fast |= a.state() == GEN && a.baud() == 921600 && b.baud() == 921600;
    }
    TEST_ASSERT_TRUE(fast);
    TEST_ASSERT_EQUAL(LINKED, a.state());
    TEST_ASSERT_EQUAL(LINKED, b.state());
    TEST_ASSERT_EQUAL_UINT32(0, a.failures + b.failures);
    TEST_ASSERT_EQUAL_UINT32(921600, pa.peak);
    TEST_ASSERT_EQUAL_UINT32(921600, pb.peak);
    TEST_ASSERT_EQUAL_UINT32(OTP_BAUD, pa.baud);
    TEST_ASSERT_EQUAL_UINT32(OTP_BAUD, pb.baud);
    TEST_ASSERT_EQUAL_UINT32(64 * 1024, a.total());
}

static void test_acceptor_grants_at_most_its_ceiling()
{
    Wire ab, ba;
    FakePort pa(&ba, &ab), pb(&ab, &ba);
    Link a(pa, 33), b(pb, 34);
    startBoth(a, b);
    TEST_ASSERT_TRUE(runUntil(a, &b, LINKED, 10000));
    Run r;
    r.baud = 9000000;
    r.kib = 8;
    a.startRun(r, now);
    for (uint32_t i = 0; i < 400 && !(a.state() == GEN && b.state() == ACCEPT); i++, now++) {
        a.poll(now);
        b.poll(now);
    }
    TEST_ASSERT_EQUAL(GEN, a.state());
    TEST_ASSERT_EQUAL_UINT32(MAX_BAUD, a.baud());
}

static void test_peer_that_never_switches_fails_and_both_return_to_base()
{
    Wire ab, ba;
    FakePort pa(&ba, &ab), pb(&ab, &ba);
    Link a(pa, 135), b(pb, 136); // seeds 35/36 hit a gate false positive at byte 480503 (0.5% of nonces)
    startBoth(a, b);
    TEST_ASSERT_TRUE(runUntil(a, &b, LINKED, 10000));
    pb.stuck = true;
    Run r;
    r.baud = 921600;
    r.kib = 64;
    a.startRun(r, now);
    for (uint32_t i = 0; i < 3000 && !(a.failures && b.failures); i++, now++) {
        a.poll(now);
        b.poll(now);
    }
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(1, a.failures);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(1, b.failures);
    TEST_ASSERT_EQUAL_UINT32(OTP_BAUD, pa.baud);
    pb.stuck = false;
    TEST_ASSERT_TRUE(runUntil(a, &b, LINKED, 20000));
}

static void test_relink_after_fault_needs_no_new_block()
{
    Wire ab, ba;
    FakePort pa(&ba, &ab), pb(&ab, &ba);
    Link a(pa, 37), b(pb, 38);
    startBoth(a, b);
    TEST_ASSERT_TRUE(runUntil(a, &b, LINKED, 10000));
    TEST_ASSERT_TRUE(a.demoDone() && b.demoDone());
    ba.q.push_back(0x55); // junk at a: a fails, b notices the silence
    a.poll(now++);
    bool block = false;
    bool relinked = false;
    for (uint32_t i = 0; i < 20000 && !relinked; i++, now++) {
        a.poll(now);
        b.poll(now);
        block |= a.state() == GEN || a.state() == ACCEPT || b.state() == GEN || b.state() == ACCEPT;
        relinked = i > 100 && a.state() == LINKED && b.state() == LINKED;
    }
    TEST_ASSERT_TRUE(relinked);
    TEST_ASSERT_FALSE(block);
}

static void test_result_and_log_lines_of_a_block()
{
    Wire w;
    FakePort p(&w, &w);
    Lines log;
    Link a(p, 1234);
    a.setLogger(&log);
    a.start(now);
    TEST_ASSERT_TRUE(runUntil(a, nullptr, LINKED, 10000));
    const Result &r = a.result();
    TEST_ASSERT_EQUAL_UINT32(1, r.seq);
    TEST_ASSERT_TRUE(r.ok);
    TEST_ASSERT_EQUAL_UINT32(BLOCK_KIB * 1024u, r.bytes);
    TEST_ASSERT_GREATER_THAN_UINT32(0, r.bytesPerSec);
    TEST_ASSERT_TRUE(log.has("link open"));
    TEST_ASSERT_TRUE(log.has("peer detected"));
    TEST_ASSERT_TRUE(log.has("block start GEN 16k 115200 baud"));
    TEST_ASSERT_TRUE(log.has("block ok GEN 16k"));
}

static void test_failure_gives_result_with_reason_and_log_line()
{
    Wire ab, ba;
    FakePort pa(&ba, &ab), pb(&ab, &ba);
    Lines log;
    Link a(pa, 33), b(pb, 44);
    a.setLogger(&log);
    startBoth(a, b);
    for (uint32_t i = 0; i < 5000 && a.state() != GEN && b.state() != GEN; i++, now++) {
        a.poll(now);
        b.poll(now);
    }
    Wire &toAcceptor = (a.state() == GEN) ? ab : ba;
    toAcceptor.corruptAt = toAcceptor.count + 1000;
    for (uint32_t i = 0; i < 3000 && !a.failures; i++, now++) {
        a.poll(now);
        b.poll(now);
    }
    TEST_ASSERT_FALSE(a.result().ok);
    TEST_ASSERT_GREATER_THAN_UINT32(0, strlen(a.result().reason));
    TEST_ASSERT_TRUE(log.has("FAIL "));
}

static void test_single_byte_window_measures_round_trips()
{
    Wire w;
    FakePort p(&w, &w);
    Link a(p, 4321);
    a.start(now);
    TEST_ASSERT_TRUE(runUntil(a, nullptr, LINKED, 10000));
    Run r;
    r.kib = 1;
    r.window = 1;
    TEST_ASSERT_TRUE(a.startRun(r, now));
    for (uint32_t i = 0; i < 20000 && a.result().seq < 2; i++, now++)
        a.poll(now);
    TEST_ASSERT_EQUAL_UINT32(2, a.result().seq);
    TEST_ASSERT_EQUAL_UINT32(1024, a.result().rtt.n);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(1000, a.result().rtt.min);
}

static unsigned count(const std::string &s, const char *what)
{
    unsigned n = 0;
    for (size_t p = s.find(what); p != std::string::npos; p = s.find(what, p + 1))
        n++;
    return n;
}

// press the key until a group runs, drive both ends until it is finished
static void runGroup(Suite &s, Link &a, Link *b, uint32_t maxMs)
{
    // a key press is refused until the heartbeat is flowing again
    for (int i = 0; i < 8000 && !s.press(now); i++, now++) {
        a.poll(now);
        if (b)
            b->poll(now);
    }
    TEST_ASSERT_TRUE(s.running());
    for (uint32_t i = 0; i < maxMs && s.running(); i++, now++) {
        a.poll(now);
        if (b)
            b->poll(now);
        s.poll(now);
    }
    TEST_ASSERT_FALSE(s.running());
}

static void test_fault_suite_passes_on_two_peers()
{
    Wire ab, ba;
    FakePort pa(&ba, &ab), pb(&ab, &ba);
    Lines log;
    Link a(pa, 301), b(pb, 302);
    Suite s(a, &log);
    startBoth(a, b);
    TEST_ASSERT_TRUE(runUntil(a, &b, LINKED, 20000));
    for (int i = 0; i < 1000; i++, now++) { // heartbeats settle
        a.poll(now);
        b.poll(now);
    }
    s.press(now); // speed group first, let it go: next press is the faults group
    for (uint32_t i = 0; i < 400000 && s.running(); i++, now++) {
        a.poll(now);
        b.poll(now);
        s.poll(now);
    }
    TEST_ASSERT_FALSE(s.running());
    TEST_ASSERT_EQUAL(Suite::FAULTS, s.next());
    log.all.clear();
    runGroup(s, a, &b, 600000);
    TEST_ASSERT_TRUE(log.has("suite faults done 13 ok 0 failed 0 skipped"));
    TEST_ASSERT_EQUAL_UINT32(13, count(log.all, " PASS"));
    TEST_ASSERT_EQUAL_UINT32(0, count(log.all, " FAIL"));
}

static void test_fault_suite_passes_in_loopback()
{
    Wire w;
    FakePort p(&w, &w);
    Lines log;
    Link a(p, 303);
    Suite s(a, &log);
    a.start(now);
    TEST_ASSERT_TRUE(runUntil(a, nullptr, LINKED, 20000));
    for (int i = 0; i < 1000; i++, now++)
        a.poll(now);
    s.press(now);
    for (uint32_t i = 0; i < 400000 && s.running(); i++, now++) {
        a.poll(now);
        s.poll(now);
    }
    log.all.clear();
    runGroup(s, a, nullptr, 600000);
    TEST_ASSERT_TRUE(log.has("suite faults done 11 ok 0 failed 2 skipped"));
}

static void test_tuning_group_reports_every_window()
{
    Wire ab, ba;
    FakePort pa(&ba, &ab), pb(&ab, &ba);
    Lines log;
    Link a(pa, 305), b(pb, 306);
    a.setLogger(&log);
    Suite s(a, &log);
    startBoth(a, b);
    TEST_ASSERT_TRUE(runUntil(a, &b, LINKED, 20000));
    for (int i = 0; i < 1000; i++, now++) {
        a.poll(now);
        b.poll(now);
    }
    for (int g = 0; g < 3; g++) // speed, faults, tuning
        runGroup(s, a, &b, 800000);
    TEST_ASSERT_TRUE(log.has("suite tuning done 5 ok 0 failed 0 skipped"));
    TEST_ASSERT_TRUE(log.has("rtt 1024 round trips"));
    TEST_ASSERT_TRUE(log.has("window 1024"));
}

// hardware log 2026-10-25: the loop turn is ~14 ms (LCD, scheduler), a 128 byte window moves ~4.4 kB/s whatever the baud, and
// the blocks of the speed group took far longer than the wire time; the suite must wait for them instead of timing out
static void test_suite_waits_for_slow_blocks()
{
    Wire ab, ba;
    FakePort pa(&ba, &ab), pb(&ab, &ba);
    Lines log;
    Link a(pa, 307), b(pb, 308);
    Suite s(a, &log);
    startBoth(a, b);
    for (uint32_t i = 0; i < 40000 && !(a.state() == LINKED && b.state() == LINKED); i++, now++)
        if (i % 14 == 0) {
            a.poll(now);
            b.poll(now);
        }
    TEST_ASSERT_EQUAL(LINKED, a.state());
    for (uint32_t i = 0; i < 3000 && !s.press(now); i++, now++)
        if (i % 14 == 0) {
            a.poll(now);
            b.poll(now);
        }
    TEST_ASSERT_TRUE(s.running());
    for (uint32_t i = 0; i < 1500000 && s.running(); i++, now++)
        if (i % 14 == 0) {
            a.poll(now);
            b.poll(now);
            s.poll(now);
        }
    TEST_ASSERT_FALSE(s.running());
    TEST_ASSERT_TRUE(log.has("suite speed done 8 ok 0 failed 0 skipped"));
}

// heartbeat spacing was +-50% of 200 ms; the hardware loop broke it, the window is now 40..1000 ms
static void test_heartbeat_gap_up_to_a_second_is_tolerated()
{
    Wire ab, ba;
    FakePort pa(&ba, &ab), pb(&ab, &ba);
    Link a(pa, 41), b(pb, 42);
    startBoth(a, b);
    TEST_ASSERT_TRUE(runUntil(a, &b, LINKED, 20000));
    for (int i = 0; i < 1000; i++, now++) {
        a.poll(now);
        b.poll(now);
    }
    uint32_t failed = a.failures + b.failures;
    ba.cut = true; // b is silent for 700 ms, then comes back
    for (int i = 0; i < 700; i++, now++) {
        a.poll(now);
        b.poll(now);
    }
    ba.cut = false;
    for (int i = 0; i < 1000; i++, now++) {
        a.poll(now);
        b.poll(now);
    }
    TEST_ASSERT_EQUAL_UINT32(failed, a.failures + b.failures);
    TEST_ASSERT_EQUAL(LINKED, a.state());
}

// the whole run through speed, faults and tuning, polled every millisecond over a wire with real byte times, must take under 30 s
static void test_all_groups_finish_in_under_30_seconds()
{
    Wire ab, ba;
    FakePort pa(&ba, &ab), pb(&ab, &ba);
    Lines log;
    Link a(pa, 401), b(pb, 402);
    a.setLogger(&log);
    Suite s(a, &log);
    startBoth(a, b);
    TEST_ASSERT_TRUE(runUntil(a, &b, LINKED, 20000));
    uint32_t total = 0;
    for (int g = 0; g < 3; g++) {
        for (int i = 0; i < 8000 && !s.press(now); i++, now++) {
            a.poll(now);
            b.poll(now);
        }
        TEST_ASSERT_TRUE(s.running());
        uint32_t t0 = now;
        for (uint32_t i = 0; i < 100000 && s.running(); i++, now++) {
            a.poll(now);
            b.poll(now);
            s.poll(now);
        }
        TEST_ASSERT_FALSE(s.running());
        total += now - t0;
    }
    if (getenv("OTP_VERBOSE"))
        printf("%s", log.all.c_str());
    char msg[64];
    snprintf(msg, sizeof(msg), "all groups took %u ms", (unsigned)total);
    TEST_MESSAGE(msg);
    TEST_ASSERT_LESS_THAN_UINT32(30000, total);
    TEST_ASSERT_EQUAL_UINT32(0, count(log.all, " FAIL"));
}

// a Cardputer with G13 jumpered to G15: one UART talks to itself. Every group must run to the end, the two speed faults that a
// single UART cannot show are skipped, nothing else may fail, and the whole run stays under 30 s
static void test_all_groups_run_to_completion_in_loopback()
{
    Wire w;
    FakePort p(&w, &w);
    Lines log;
    Link a(p, 501);
    a.setLogger(&log);
    Suite s(a, &log);
    a.start(now);
    TEST_ASSERT_TRUE(runUntil(a, nullptr, LINKED, 20000));
    uint32_t total = 0;
    for (int g = 0; g < 3; g++) {
        for (int i = 0; i < 8000 && !s.press(now); i++, now++)
            a.poll(now);
        TEST_ASSERT_TRUE(s.running());
        uint32_t t0 = now;
        for (uint32_t i = 0; i < 100000 && s.running(); i++, now++) {
            a.poll(now);
            s.poll(now);
        }
        TEST_ASSERT_FALSE(s.running());
        total += now - t0;
    }
    if (getenv("OTP_VERBOSE"))
        printf("%s", log.all.c_str());
    TEST_ASSERT_TRUE(log.has("suite speed done 8 ok 0 failed 0 skipped"));
    TEST_ASSERT_TRUE(log.has("suite faults done 11 ok 0 failed 2 skipped"));
    TEST_ASSERT_TRUE(log.has("suite tuning done 5 ok 0 failed 0 skipped"));
    TEST_ASSERT_EQUAL_UINT32(0, count(log.all, "case ") - count(log.all, " PASS") - count(log.all, "skipped: loopback"));
    TEST_ASSERT_LESS_THAN_UINT32(30000, total);
}

void setUp(void) {}

void tearDown(void) {}

extern "C" {
void setup()
{
    initializeTestEnvironment();
    UNITY_BEGIN();
    RUN_TEST(test_loopback_reaches_linked);
    RUN_TEST(test_two_peers_pick_roles_and_link);
    RUN_TEST(test_corrupted_byte_in_transfer_fails_both_then_relinks);
    RUN_TEST(test_cut_wire_in_transfer_times_out_both);
    RUN_TEST(test_missed_heartbeat_when_linked_resets_both);
    RUN_TEST(test_junk_byte_when_linked_resets);
    RUN_TEST(test_key_starts_new_handshake_without_failure);
    RUN_TEST(test_simultaneous_requests_resolve_to_one_generator);
    RUN_TEST(test_entropy_gate_passes_xorshift_blocks);
    RUN_TEST(test_entropy_gate_trips_on_structure);
    RUN_TEST(test_acceptor_rejects_structured_block);
    RUN_TEST(test_speed_negotiated_block_runs_fast_and_returns_to_base);
    RUN_TEST(test_acceptor_grants_at_most_its_ceiling);
    RUN_TEST(test_peer_that_never_switches_fails_and_both_return_to_base);
    RUN_TEST(test_relink_after_fault_needs_no_new_block);
    RUN_TEST(test_result_and_log_lines_of_a_block);
    RUN_TEST(test_failure_gives_result_with_reason_and_log_line);
    RUN_TEST(test_single_byte_window_measures_round_trips);
    RUN_TEST(test_fault_suite_passes_on_two_peers);
    RUN_TEST(test_fault_suite_passes_in_loopback);
    RUN_TEST(test_tuning_group_reports_every_window);
    RUN_TEST(test_suite_waits_for_slow_blocks);
    RUN_TEST(test_heartbeat_gap_up_to_a_second_is_tolerated);
    RUN_TEST(test_all_groups_finish_in_under_30_seconds);
    RUN_TEST(test_all_groups_run_to_completion_in_loopback);
    exit(UNITY_END());
}

void loop() {}
}
