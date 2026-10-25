#if defined(OTP_APP) || defined(PIO_UNIT_TESTING)

#include "OtpLink.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// wire format, states and timings: design-docs SERIAL_PROTOCOL.md
// frame: [SYNC][type][payload][crc8 over type+payload], HB has no payload and no crc
// block bytes after SPD are raw, no framing, the echo is the check
namespace otp
{
static uint32_t elapsed(uint32_t now, uint32_t since)
{
    return now - since;
}

// poly 0x07, init 0, bitwise: frames are tiny and rare
static uint8_t crc8(const uint8_t *p, size_t n)
{
    uint8_t c = 0;
    while (n--) {
        c ^= *p++;
        for (int i = 0; i < 8; i++)
            c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
    }
    return c;
}

static uint32_t le32(const uint8_t *p)
{
    return p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

void Rows::put(const char *fmt, ...)
{
    cur = (cur + 1) % N;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text[cur], W, fmt, ap);
    va_end(ap);
    rev++;
}

void Rows::clear()
{
    memset(text, 0, sizeof(text));
    rev++;
}

uint32_t Link::random()
{
    return xorshift8(rng) | (uint32_t)xorshift8(rng) << 8 | (uint32_t)xorshift8(rng) << 16 | (uint32_t)xorshift8(rng) << 24;
}

void Link::setBaud(uint32_t baud)
{
    if (baud == curBaud)
        return;
    port.setBaud(baud);
    curBaud = baud;
}

// host log line, formatted only when somebody listens
void Link::note(const char *fmt, ...)
{
    if (!logger)
        return;
    char s[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s, sizeof(s), fmt, ap);
    va_end(ap);
    logger->line(s);
}

// close the books on one run: result for the caller, one log line (block end or error)
void Link::endRun(bool ok, const char *reason, uint32_t now)
{
    bool data = state_ == GEN || state_ == ACCEPT;
    Result &r = result_;
    unsigned seq = r.seq + 1;
    r = Result();
    r.seq = seq;
    r.ok = ok;
    snprintf(r.reason, sizeof(r.reason), "%s", reason);
    r.baud = run_.baud;
    r.kib = run_.kib;
    r.window = run_.window;
    r.gen = gen;
    r.bytes = data ? doneCnt : 0;
    r.ms = data ? elapsed(now, runStart) : 0;
    r.bytesPerSec = r.ms ? (uint32_t)((uint64_t)r.bytes * 1000 / r.ms) : 0;
    r.gapUs = gapUs;
    r.ringHigh = ringHigh;
    r.errs = port.errors() - errsAtStart;
    r.rtt = rtt;
}

// data phase bookkeeping, once per poll: loop gap and RX ring fill
void Link::sampleLoop()
{
    uint32_t t = port.us();
    if (lastPollUs && t - lastPollUs > gapUs)
        gapUs = t - lastPollUs;
    lastPollUs = t;
    int av = port.available();
    if (av > 0 && (unsigned)av > ringHigh)
        ringHigh = av;
}

void Link::start(uint32_t now)
{
    // the module reopens the UART at the base speed, after a failure it is there already
    curBaud = OTP_BAUD;
    glitching = false;

    note(opened ? "idle: silence over, heartbeat" : "link open, heartbeat");
    if (!opened) {
        opened = true;
        linkOpenAt = now;
    }
    state_ = BEAT;
    lastBeat = now - BEAT_MS;
    hbValid = false;
    pos = 0;
    tx.clear();
    rx.put("start");
}

void Link::fail(uint32_t now, const char *fmt, ...)
{
    char msg[Rows::W];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    rx.put("FAIL %s", msg);

    // result and log line first: they need the state the failure interrupted
    sleepFor = SLEEP_MIN_MS + random() % SLEEP_SPAN_MS;
    endRun(false, msg, now);
    note("FAIL %s | %uk done, %ums, %u baud, silent %ums", msg, (unsigned)(result_.bytes / 1024), (unsigned)result_.ms,
         (unsigned)curBaud, (unsigned)sleepFor);

    // stop talking at once; silence is what resyncs the peer (its own timeout fires within 600 ms)
    sleepAt = now;
    tx.put("silent %ums", (unsigned)sleepFor);
    state_ = SLEEP;
    hbValid = false;
    pos = 0;
    failures++;

    // always wake up at the base speed, the peer does the same after its own timeout
    if (glitching) {
        // fault GLITCH left the port at 9600, curBaud does not know
        port.setBaud(curBaud);
        glitching = false;
    }
    setBaud(OTP_BAUD);
}

bool Link::send(const uint8_t *p, size_t n, uint32_t now)
{
    if (port.writable() < (int)n) {
        fail(now, "tx full");
        return false;
    }
    for (size_t i = 0; i < n; i++)
        port.write(p[i]);
    return true;
}

bool Link::sendNonce(uint8_t type, uint32_t nonce, uint32_t now)
{
    // REQ and ACC share one layout
    uint8_t f[7] = {
        SYNC,
        type,
        (uint8_t)nonce,
        (uint8_t)(nonce >> 8),
        (uint8_t)(nonce >> 16),
        (uint8_t)(nonce >> 24),
        0, // crc8
    };
    f[6] = crc8(f + 1, 5);
    return send(f, sizeof(f), now);
}

bool Link::sendSpd(uint32_t baud, unsigned kib, bool dump, uint32_t now)
{
    // generator asks, acceptor answers with the same layout and the speed it grants
    uint8_t f[10] = {
        SYNC,
        T_SPD,
        (uint8_t)baud,
        (uint8_t)(baud >> 8),
        (uint8_t)(baud >> 16),
        (uint8_t)(baud >> 24),
        (uint8_t)kib,
        (uint8_t)(kib >> 8),
        (uint8_t)(dump ? 1 : 0), // flags: bit 0 hexdump
        0,                       // crc8
    };
    f[9] = crc8(f + 1, 8);
    return send(f, sizeof(f), now);
}

void Link::sendReq(uint32_t now)
{
    myNonce = random();
    if (!sendNonce(T_REQ, myNonce, now))
        return;
    tx.put("REQ %08x", (unsigned)myNonce);
    reqAt = now;
    state_ = REQ;
}

bool Link::startRun(const Run &r, uint32_t now)
{
    if (state_ != LINKED)
        return false;
    want = r;

    // these two break the idle link instead of starting a block; the peer fails and returns within the heartbeat window, so
    // the suite judges them by the longest heartbeat gap seen afterwards
    maxHbGap_ = 0;
    if (r.fault == STRAY) {
        const uint8_t junk = 0x55;
        tx.put("fault: stray byte");
        send(&junk, 1, now);
        return true;
    }
    if (r.fault == SKIP_HB) {
        tx.put("fault: skip heartbeats");
        skipHb = true;
        skipAt = now;
        return true;
    }

    tx.put("key: new handshake");
    sendReq(now);
    return true;
}

void Link::bad(const char *what, uint32_t now)
{
    // before the handshake garbage is normal (boot noise, a peer mid-switch), after it any garbage is a fault
    if (state_ == BEAT)
        rx.put("ignored: %s", what);
    else
        fail(now, "%s", what);
}

// ACC and SPD are exchanged at 115200; the data phase starts after SWITCH at the granted speed
void Link::beginSwitch(State next, uint32_t now)
{
    switchTo = next;
    switchAt = now;
    state_ = SWITCH;
}

void Link::beginData(State s, uint32_t now)
{
    state_ = s;
    sentCnt = doneCnt = 0;
    txN = rxN = 0;
    lastProgress = lastLog = now;
    blockBytes = run_.kib * 1024u;

    // both ends derive the stream from the nonce of the REQ
    sendS = Stream(run_.src, myNonce);
    checkS = sendS;
    stalled = glitching = false;
    pos = 0;
    gate.reset();

    // fresh numbers for this run
    runStart = now;
    lastPollUs = gapUs = 0;
    ringHigh = 0;
    rtt = Rtt();
    errsAtStart = port.errors();
    note("block start %s %uk %u baud window %u nonce %08x", gen ? "GEN" : "ACC", run_.kib, (unsigned)run_.baud, run_.window,
         (unsigned)myNonce);
}

// SPD arrives in NEGO only: acceptor gets the question, generator gets the answer
void Link::onSpd(uint32_t now)
{
    uint32_t baud = le32(buf + 2);
    unsigned kib = buf[6] | buf[7] << 8;
    bool dump = buf[8] & 1;
    if (baud == 0 || kib == 0)
        return fail(now, "bad SPD");

    if (gen) {
        // answer: never faster than asked
        if (baud > want.baud)
            return fail(now, "SPD %u > %u", (unsigned)baud, (unsigned)want.baud);
        rx.put("SPD %u ok", (unsigned)baud);
        run_ = want;
        run_.baud = baud;
        tx.put("GEN %uk", kib);
        return beginSwitch(GEN, now);
    }

    // question: grant the asked speed up to our ceiling, answer, wait until it has left the wire
    uint32_t granted = baud < MAX_BAUD ? baud : MAX_BAUD;
    rx.put("SPD %u %uk", (unsigned)baud, kib);
    if (!sendSpd(granted, kib, dump, now))
        return;
    port.flush();
    run_.baud = granted;
    run_.kib = kib;
    run_.dump = dump;
    tx.put("ACC, echoing");
    beginSwitch(ACCEPT, now);
}

void Link::onFrame(uint8_t type, uint32_t nonce, uint32_t now)
{
    if (type == T_HB) {
        if (state_ == BEAT) {
            rx.put("HB: peer is there");
            note("peer detected (HB) %ums after open, %u baud", (unsigned)elapsed(now, linkOpenAt), (unsigned)curBaud);
            if (demoDone_) {
                // link proven once already: heartbeat is enough, no block after a fault
                state_ = LINKED;
                lastHb = now;
                hbValid = true;
                enteredAt = now;
            } else {
                want = Run();
                want.dump = true;
                sendReq(now);
            }
        } else if (state_ == LINKED) {
            // heartbeat spacing must stay inside HB_MIN_MS..HB_MAX_MS
            uint32_t dt = elapsed(now, lastHb);
            if (hbValid && dt > maxHbGap_)
                maxHbGap_ = dt;
            if (hbValid && (dt < HB_MIN_MS || dt > HB_MAX_MS))
                return fail(now, "HB after %ums", (unsigned)dt);
            lastHb = now;
            hbValid = true;
            hbGapUs = 0;
            if (elapsed(now, lastLog) >= 1000) {
                lastLog = now;
                rx.put("HB %ums", (unsigned)dt);
            }
        }
    } else if (type == T_REQ) {
        if (state_ == REQ && nonce == myNonce) {
            // our own REQ came back: loopback, we are generator and acceptor in one
            rx.put("own REQ back: loopback");
            gen = true;
            loop_ = true;
            negoAt = now;
            state_ = NEGO;
            sendSpd(want.baud, want.kib, want.dump, now);
        } else if (state_ == BEAT || state_ == LINKED || (state_ == REQ && nonce > myNonce)) {
            // peer generates; with two REQs in the air the higher nonce wins
            rx.put("REQ %08x", (unsigned)nonce);
            if (state_ == BEAT)
                note("peer detected (REQ) %ums after open, %u baud", (unsigned)elapsed(now, linkOpenAt), (unsigned)curBaud);
            if (!sendNonce(T_ACC, nonce, now))
                return;
            myNonce = nonce;
            gen = false;
            loop_ = false;
            negoAt = now;
            run_ = Run();
            state_ = NEGO;
        } else {
            rx.put("REQ %08x lost tie", (unsigned)nonce);
        }
    } else if (type == T_ACC) {
        if (state_ == REQ && nonce == myNonce) {
            rx.put("ACC %08x", (unsigned)nonce);
            gen = true;
            loop_ = false;
            negoAt = now;
            state_ = NEGO;
            sendSpd(want.baud, want.kib, want.dump, now);
        } else if (state_ != BEAT && state_ != REQ) {
            fail(now, "stray ACC");
        }
    } else if (type == T_SPD) {
        if (state_ == NEGO)
            onSpd(now);
        else if (state_ != BEAT && state_ != REQ)
            fail(now, "stray SPD");
    }
}

// frame parser, one byte at a time: SYNC, type, then a fixed length per type
void Link::onByte(uint8_t c, uint32_t now)
{
    if (!active(now))
        rx.put("Activity! first 0x%02x", c);
    everRx = true;
    lastRx = now;

    if (pos == 0) {
        if (c != SYNC)
            return bad("junk byte", now);
        buf[pos++] = c;
        frameAt = now;
        return;
    }
    buf[pos++] = c;

    // length by type: HB 2, REQ and ACC 7, SPD 10
    static const uint8_t lenOf[] = {0, 2, 7, 7, 10};
    if (pos == 2) {
        if (c == 0 || c > T_SPD) {
            pos = 0;
            return bad("unknown frame", now);
        }
        if (c == T_HB) {
            pos = 0;
            return onFrame(T_HB, 0, now);
        }
    } else if (pos == lenOf[buf[1]]) {
        pos = 0;
        if (crc8(buf + 1, lenOf[buf[1]] - 2) != buf[lenOf[buf[1]] - 1])
            return bad("frame crc", now);
        onFrame(buf[1], le32(buf + 2), now);
    }
}

void Link::hexLine(Rows &r, const char *tag, const uint8_t *ring, uint32_t n)
{
    // `178k |72 6f 6d 20 74 65|` the six newest bytes
    char hex[24] = "";
    uint32_t cnt = n < 6 ? n : 6;
    for (uint32_t i = 0; i < cnt; i++)
        snprintf(hex + i * 3, sizeof(hex) - i * 3, "%02x ", ring[(n - cnt + i) % 6]);
    if (cnt)
        hex[cnt * 3 - 1] = '\0';
    r.put("%s %uk |%s|", tag, (unsigned)(n / 1024), hex);
}

void Link::finish(uint32_t now)
{
    endRun(true, "", now);
    const Result &r = result_;
    note("block ok %s %uk %ums %u B/s window %u gap %uus ring %u err ovf %u/%u frm %u par %u brk %u", gen ? "GEN" : "ACC",
         (unsigned)(r.bytes / 1024), (unsigned)r.ms, (unsigned)r.bytesPerSec, (unsigned)r.window, (unsigned)r.gapUs, r.ringHigh,
         r.errs.fifo, r.errs.ring, r.errs.frame, r.errs.parity, r.errs.brk);
    if (r.rtt.n)
        note("rtt %u round trips min %u avg %u max %u us", (unsigned)r.rtt.n, (unsigned)r.rtt.min, (unsigned)r.rtt.avg(),
             (unsigned)r.rtt.max);
    rx.put("block of %uk ok", (unsigned)(doneCnt / 1024));
    tx.put("block done: connected");
    state_ = LINKED;
    enteredAt = lastBeat = now;
    hbValid = false;
    demoDone_ = true;

    // last echo byte must be on the wire before the speed drops back
    port.flush();
    setBaud(OTP_BAUD);
}

// fault hits the byte at faultAt; false means do not send it
bool Link::inject(uint8_t &b, uint32_t now)
{
    switch (run_.fault) {
    case FLIP:
        b ^= 0x40;
        break;
    case DROP:
        return false;
    case STALL:
        stalled = true;
        return false;
    case GLITCH:
        port.setBaud(9600);
        glitching = true;
        glitchAt = now;
        break;
    default:
        break;
    }
    return true;
}

void Link::pumpGen(uint32_t now)
{
    sampleLoop();
    int c;
    for (int n = 0; n < 256 && doneCnt < blockBytes && (c = port.read()) >= 0; n++) {
        everRx = true;
        lastRx = now;
        uint8_t want = checkS.next();
        if (c != want)
            return fail(now, "echo %u: %02x not %02x", (unsigned)doneCnt, c, want);

        // echo matches, but is the stream itself still random
        if (!gate.feed((uint8_t)c))
            return fail(now, "low entropy at %u", (unsigned)doneCnt);

        // window 1 is the round trip probe: byte out to byte back, in microseconds
        if (run_.window == 1)
            rtt.add(port.us() - sendUs);

        rxRing[rxN++ % 6] = (uint8_t)c;
        doneCnt++;
        lastProgress = now;
    }
    if (doneCnt >= blockBytes)
        return finish(now);

    // fault GLITCH: back to the run speed after 100 ms
    if (glitching && elapsed(now, glitchAt) >= 100) {
        port.setBaud(curBaud);
        glitching = false;
    }

    // keep at most `window` bytes unechoed in flight, the UART buffers are small
    for (int n = 0; n < 256 && !stalled && sentCnt < blockBytes && sentCnt - doneCnt < run_.window && port.writable() > 0; n++) {
        uint8_t b = sendS.next();
        bool send = sentCnt != run_.faultAt || inject(b, now);
        if (send) {
            port.write(b);
            sendUs = port.us();
            txRing[txN++ % 6] = b;
        }
        sentCnt++;
    }
    if (elapsed(now, lastProgress) >= STALL_MS)
        return fail(now, "stall at %u", (unsigned)doneCnt);
    if (run_.dump && elapsed(now, lastLog) >= 250) {
        lastLog = now;
        hexLine(tx, "TX", txRing, txN);
        hexLine(rx, "RX", rxRing, rxN);
    }
}

void Link::pumpAccept(uint32_t now)
{
    sampleLoop();
    int c;
    for (int n = 0; n < 256 && port.writable() > 0 && (c = port.read()) >= 0; n++) {
        everRx = true;
        lastRx = now;

        // acceptor judges the stream before echoing it back
        if (!gate.feed((uint8_t)c))
            return fail(now, "low entropy at %u", (unsigned)doneCnt);

        port.write((uint8_t)c);
        rxRing[rxN++ % 6] = (uint8_t)c;
        doneCnt++;
        lastProgress = now;
        if (doneCnt >= blockBytes)
            return finish(now);
    }
    if (elapsed(now, lastProgress) >= STALL_MS)
        return fail(now, "stall at %u", (unsigned)doneCnt);
    if (run_.dump && elapsed(now, lastLog) >= 250) {
        lastLog = now;
        hexLine(rx, "RX", rxRing, rxN);
        hexLine(tx, "TX", rxRing, rxN);
    }
}

void Link::poll(uint32_t now)
{
    // longest time between two polls since the peer's last heartbeat, to explain a missed one
    uint32_t us = port.us();
    if (prevPollUs && us - prevPollUs > hbGapUs)
        hbGapUs = us - prevPollUs;
    prevPollUs = us;

    switch (state_) {
    case SLEEP:
        while (port.read() >= 0) {
            if (!active(now))
                rx.put("Activity! (muted)");
            everRx = true;
            lastRx = now;
        }
        if (elapsed(now, sleepAt) >= sleepFor)
            start(now);
        return;
    case SWITCH:
        // silent; the acceptor changes speed first, the generator a bit later, then both drop what the change left in RX
        if (elapsed(now, switchAt) >= (gen ? SWITCH_GEN_MS : SWITCH_ACC_MS)) {
            if (!(gen && run_.fault == NO_SWITCH))
                setBaud(run_.baud);
            while (port.read() >= 0)
                ;
            beginData(switchTo, now);
        }
        return;
    case GEN:
        return pumpGen(now);
    case ACCEPT:
        return pumpAccept(now);
    default:
        break;
    }

    if ((state_ == BEAT || state_ == LINKED) && elapsed(now, lastBeat) >= BEAT_MS) {
        lastBeat = now;
        const uint8_t hb[2] = {SYNC, T_HB};
        // fault SKIP_HB: silent for longer than the peer tolerates
        if (skipHb && elapsed(now, skipAt) > HB_MAX_MS + BEAT_MS)
            skipHb = false;
        if (!skipHb)
            send(hb, sizeof(hb), now);
    }
    int c;
    bool got = false;
    for (int n = 0; n < 64 && framing() && (c = port.read()) >= 0; n++) {
        got = true;
        onByte((uint8_t)c, now);
    }
    if (!framing())
        return;
    if (!got && pos && elapsed(now, frameAt) > FRAME_MS) {
        pos = 0;
        bad("partial frame", now);
    } else if (state_ == REQ && elapsed(now, reqAt) >= REQ_MS) {
        fail(now, "no ACC");
    } else if (state_ == NEGO && elapsed(now, negoAt) >= NEGO_MS) {
        fail(now, "no SPD");
    } else if (state_ == LINKED) {
        uint32_t limit = hbValid ? HB_MAX_MS : HB_FIRST_MS;
        if (elapsed(now, hbValid ? lastHb : enteredAt) > limit)
            fail(now, "HB missed, loop gap %ums", (unsigned)(hbGapUs / 1000));
    }
}
} // namespace otp

#endif
