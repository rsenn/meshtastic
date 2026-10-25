#if defined(OTP_APP) || defined(PIO_UNIT_TESTING)

#include "OtpLink.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace otp
{
static uint32_t elapsed(uint32_t now, uint32_t since)
{
    return now - since;
}

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

// xorshift32: stand-in for real entropy, one byte per step.
static uint8_t nextByte(uint32_t &x)
{
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return (uint8_t)(x >> 8);
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
    return nextByte(rng) | (uint32_t)nextByte(rng) << 8 | (uint32_t)nextByte(rng) << 16 | (uint32_t)nextByte(rng) << 24;
}

void Link::start(uint32_t now)
{
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
    sleepAt = now;
    sleepFor = SLEEP_MIN_MS + random() % SLEEP_SPAN_MS;
    tx.put("silent %ums", (unsigned)sleepFor);
    state_ = SLEEP;
    hbValid = false;
    pos = 0;
    failures++;
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
    uint8_t f[7] = {SYNC, type, (uint8_t)nonce, (uint8_t)(nonce >> 8), (uint8_t)(nonce >> 16), (uint8_t)(nonce >> 24), 0};
    f[6] = crc8(f + 1, 5);
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

void Link::restart(uint32_t now)
{
    if (state_ == LINKED) {
        tx.put("key: new handshake");
        sendReq(now);
    }
}

void Link::bad(const char *what, uint32_t now)
{
    if (state_ == BEAT)
        rx.put("ignored: %s", what);
    else
        fail(now, "%s", what);
}

void Link::beginData(State s, uint32_t now)
{
    state_ = s;
    sentCnt = doneCnt = 0;
    txN = rxN = 0;
    lastProgress = lastLog = now;
    sendRng = checkRng = myNonce ? myNonce : 1;
    pos = 0;
}

void Link::onFrame(uint8_t type, uint32_t nonce, uint32_t now)
{
    if (type == T_HB) {
        if (state_ == BEAT) {
            rx.put("HB: peer is there");
            sendReq(now);
        } else if (state_ == LINKED) {
            uint32_t dt = elapsed(now, lastHb);
            if (hbValid && (dt < BEAT_MS / 2 || dt > BEAT_MS * 3 / 2))
                return fail(now, "HB after %ums", (unsigned)dt);
            lastHb = now;
            hbValid = true;
            if (elapsed(now, lastLog) >= 1000) {
                lastLog = now;
                rx.put("HB %ums", (unsigned)dt);
            }
        }
    } else if (type == T_REQ) {
        if (state_ == REQ && nonce == myNonce) {
            rx.put("own REQ back: loopback");
            tx.put("GEN %uk", (unsigned)(BLOCK_BYTES / 1024));
            beginData(GEN, now);
        } else if (state_ == BEAT || state_ == LINKED || (state_ == REQ && nonce > myNonce)) {
            rx.put("REQ %08x", (unsigned)nonce);
            if (!sendNonce(T_ACC, nonce, now))
                return;
            tx.put("ACC, echoing");
            myNonce = nonce;
            beginData(ACCEPT, now);
        } else {
            rx.put("REQ %08x lost tie", (unsigned)nonce);
        }
    } else if (type == T_ACC) {
        if (state_ == REQ && nonce == myNonce) {
            rx.put("ACC %08x", (unsigned)nonce);
            tx.put("GEN %uk", (unsigned)(BLOCK_BYTES / 1024));
            beginData(GEN, now);
        } else if (state_ != BEAT && state_ != REQ) {
            fail(now, "stray ACC");
        }
    }
}

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
    if (pos == 2) {
        if (c == T_HB) {
            pos = 0;
            return onFrame(T_HB, 0, now);
        }
        if (c != T_REQ && c != T_ACC) {
            pos = 0;
            return bad("unknown frame", now);
        }
    } else if (pos == 7) {
        pos = 0;
        if (crc8(buf + 1, 5) != buf[6])
            return bad("frame crc", now);
        uint32_t n = buf[2] | (uint32_t)buf[3] << 8 | (uint32_t)buf[4] << 16 | (uint32_t)buf[5] << 24;
        onFrame(buf[1], n, now);
    }
}

void Link::hexLine(Rows &r, const char *tag, const uint8_t *ring, uint32_t n)
{
    char hex[24] = "";
    uint32_t cnt = n < 6 ? n : 6;
    for (uint32_t i = 0; i < cnt; i++)
        snprintf(hex + i * 3, sizeof(hex) - i * 3, "%02x ", ring[(n - cnt + i) % 6]);
    r.put("%s %uk %s", tag, (unsigned)(n / 1024), hex);
}

void Link::finish(uint32_t now)
{
    rx.put("block of %uk ok", (unsigned)(doneCnt / 1024));
    tx.put("block done: connected");
    state_ = LINKED;
    enteredAt = lastBeat = now;
    hbValid = false;
}

void Link::pumpGen(uint32_t now)
{
    int c;
    for (int n = 0; n < 256 && doneCnt < BLOCK_BYTES && (c = port.read()) >= 0; n++) {
        everRx = true;
        lastRx = now;
        uint8_t want = nextByte(checkRng);
        if (c != want)
            return fail(now, "echo %u: %02x not %02x", (unsigned)doneCnt, c, want);
        rxRing[rxN++ % 6] = (uint8_t)c;
        doneCnt++;
        lastProgress = now;
    }
    if (doneCnt >= BLOCK_BYTES)
        return finish(now);
    for (int n = 0; n < 256 && sentCnt < BLOCK_BYTES && sentCnt - doneCnt < WINDOW && port.writable() > 0; n++) {
        uint8_t b = nextByte(sendRng);
        port.write(b);
        txRing[txN++ % 6] = b;
        sentCnt++;
    }
    if (elapsed(now, lastProgress) >= STALL_MS)
        return fail(now, "stall at %u", (unsigned)doneCnt);
    if (elapsed(now, lastLog) >= 250) {
        lastLog = now;
        hexLine(tx, "TX", txRing, txN);
        hexLine(rx, "RX", rxRing, rxN);
    }
}

void Link::pumpAccept(uint32_t now)
{
    int c;
    for (int n = 0; n < 256 && port.writable() > 0 && (c = port.read()) >= 0; n++) {
        everRx = true;
        lastRx = now;
        port.write((uint8_t)c);
        rxRing[rxN++ % 6] = (uint8_t)c;
        doneCnt++;
        lastProgress = now;
        if (doneCnt >= BLOCK_BYTES)
            return finish(now);
    }
    if (elapsed(now, lastProgress) >= STALL_MS)
        return fail(now, "stall at %u", (unsigned)doneCnt);
    if (elapsed(now, lastLog) >= 250) {
        lastLog = now;
        hexLine(rx, "RX", rxRing, rxN);
        hexLine(tx, "TX", rxRing, rxN);
    }
}

void Link::poll(uint32_t now)
{
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
        send(hb, sizeof(hb), now);
    }
    int c;
    bool got = false;
    for (int n = 0; n < 64 && (state_ == BEAT || state_ == REQ || state_ == LINKED) && (c = port.read()) >= 0; n++) {
        got = true;
        onByte((uint8_t)c, now);
    }
    if (state_ != BEAT && state_ != REQ && state_ != LINKED)
        return;
    if (!got && pos && elapsed(now, frameAt) > FRAME_MS) {
        pos = 0;
        bad("partial frame", now);
    } else if (state_ == REQ && elapsed(now, reqAt) >= REQ_MS) {
        fail(now, "no ACC");
    } else if (state_ == LINKED) {
        uint32_t limit = hbValid ? BEAT_MS * 3 / 2 : BEAT_MS * 3;
        if (elapsed(now, hbValid ? lastHb : enteredAt) > limit)
            fail(now, "HB missed");
    }
}
} // namespace otp

#endif
