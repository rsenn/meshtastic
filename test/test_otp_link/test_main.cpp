#include "TestUtil.h"
#include "modules/optional/Otp/OtpLink.h"
#include <deque>
#include <unity.h>

// Serial link state machine of the OTP demo screen: heartbeat, handshake with role pick, 512k echo
// exchange, and bail-out (silence, then relink) on any glitch. The wire is simulated, one tick = 1 ms.
// Regression guarded: a failure must leave both ends silent and able to link again.

using namespace otp;

struct Wire {
    std::deque<uint8_t> q;
    bool cut = false;      // drop everything written
    int corruptAt = -1;    // flip the byte with this running index
    int count = 0;
};

struct FakePort : Port {
    Wire *in, *out;
    FakePort(Wire *in, Wire *out) : in(in), out(out) {}
    int read() override
    {
        if (in->q.empty())
            return -1;
        int c = in->q.front();
        in->q.pop_front();
        return c;
    }
    int writable() override { return 1000; }
    void write(uint8_t b) override
    {
        if (out->count++ == out->corruptAt)
            b ^= 0x40;
        if (!out->cut)
            out->q.push_back(b);
    }
};

static uint32_t now = 1000;

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
    TEST_ASSERT_EQUAL_UINT32(BLOCK_BYTES, a.done());
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
    TEST_ASSERT_EQUAL(SLEEP, a.state());
    TEST_ASSERT_EQUAL(SLEEP, b.state());
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
    for (uint32_t i = 0; i < 400; i++, now++) {
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
    for (uint32_t i = 0; i < 20 && a.state() == REQ; i++, now++) {
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
    exit(UNITY_END());
}

void loop() {}
}
