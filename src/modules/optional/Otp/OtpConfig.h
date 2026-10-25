#pragma once

#include <stdint.h>

#if defined(M5STACK_CARDPUTER_ADV)
#define OTP_UART_TX 13
#define OTP_UART_RX 15
#elif defined(T_LORA_PAGER)
#define OTP_UART_TX 43
#define OTP_UART_RX 44
#endif

#define OTP_BAUD 115200
#define OTP_RX_RING 2048 // driver RX ring, must hold the largest window of the tuning sweep (1024) plus a slow loop turn
#define OTP_TX_RING 1024

// shown while nothing was ever received (RX half), and what to do when wired but silent (TX half)
#if defined(M5STACK_CARDPUTER_ADV)
#define OTP_PIN_HINT1 "cap off: G13=TX  G15=RX"
#define OTP_PIN_HINT2 "cross TX-RX, join GND"
#elif defined(T_LORA_PAGER)
#define OTP_PIN_HINT1 "12-pin header: TX = G43   RX = G44"
#define OTP_PIN_HINT2 "cross TX to RX, join GND"
#endif
#define OTP_WIRED_HINT1 "wired, no Activity! in 1 s:"
#define OTP_WIRED_HINT2 "unplug, recheck wiring"

namespace otp
{
constexpr uint8_t SYNC = 0xA5;
constexpr uint8_t T_HB = 0x01;  // [SYNC][T_HB]
constexpr uint8_t T_REQ = 0x02; // [SYNC][T_REQ][nonce:4 LE][crc8]: "I generate"
constexpr uint8_t T_ACC = 0x03; // [SYNC][T_ACC][nonce:4 LE][crc8]: "accepted, echoing"
constexpr uint8_t T_SPD = 0x04; // [SYNC][T_SPD][baud:4 LE][kib:2 LE][flags][crc8]: block parameters, asked then granted

constexpr uint32_t BEAT_MS = 200;    // heartbeat cycle
constexpr uint32_t HB_MIN_MS = 40;   // peer heartbeat spacing accepted: loosened from +-50% (developers, 2026-10-25)
constexpr uint32_t HB_MAX_MS = 1000; // longer than this the peer is gone
constexpr uint32_t HB_FIRST_MS = 1500; // first heartbeat after entering LINKED
constexpr uint32_t FRAME_MS = 50;    // a started frame must complete within this
constexpr uint32_t REQ_MS = 400;     // REQ without ACC
constexpr uint32_t STALL_MS = 120;   // data phase without progress; the link task polls every ms, so this is many polls
constexpr uint32_t ACTIVE_MS = 1500; // RX shows "Activity!" this long after the last byte
constexpr uint32_t SLEEP_MIN_MS = 250;
constexpr uint32_t SLEEP_SPAN_MS = 250; // sleep is 250..500 ms: a peer still in a block fails on our first heartbeat anyway
constexpr unsigned NEGO_MS = 300;       // SPD must follow ACC and be answered within this
constexpr unsigned SWITCH_ACC_MS = 10;  // acceptor switches this long after its SPD reply left the wire
constexpr unsigned SWITCH_GEN_MS = 40;  // generator waits longer, so the acceptor is always at the new speed first
constexpr uint32_t MAX_BAUD = 5000000;  // ESP32-S3 UART ceiling, the acceptor never grants more
constexpr unsigned BLOCK_KIB = 16; // demo block: 1.4 s at 115200, tests scale the size with the speed
constexpr uint32_t WINDOW = 512; // unechoed bytes in flight; the RX ring (2048) holds it with room
} // namespace otp
