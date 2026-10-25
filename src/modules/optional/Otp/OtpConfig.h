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

namespace otp
{
constexpr uint8_t SYNC = 0xA5;
constexpr uint8_t T_HB = 0x01;  // [SYNC][T_HB]
constexpr uint8_t T_REQ = 0x02; // [SYNC][T_REQ][nonce:4 LE][crc8]: "I generate"
constexpr uint8_t T_ACC = 0x03; // [SYNC][T_ACC][nonce:4 LE][crc8]: "accepted, echoing"

constexpr uint32_t BEAT_MS = 200;    // heartbeat cycle; the peer's must arrive within +-50%
constexpr uint32_t FRAME_MS = 50;    // a started frame must complete within this
constexpr uint32_t REQ_MS = 600;     // REQ without ACC
constexpr uint32_t STALL_MS = 500;   // data phase without progress
constexpr uint32_t ACTIVE_MS = 1500; // RX shows "Activity!" this long after the last byte
constexpr uint32_t SLEEP_MIN_MS = 1000;
constexpr uint32_t SLEEP_SPAN_MS = 2000; // sleep is 1..3 s, longer than any peer detection time above
constexpr uint32_t BLOCK_BYTES = 512u * 1024u;
constexpr uint32_t WINDOW = 128; // unechoed bytes in flight
} // namespace otp
