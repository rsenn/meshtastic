#include "OtpModule.h"

#if defined(OTP_APP) && defined(OTP_UART_TX) && HAS_SCREEN

#include "esp_random.h"
#include "graphics/ScreenFonts.h"
#include "mesh/Throttle.h"
#include <HardwareSerial.h>

int OtpModule::Uart::read()
{
    return Serial2.read();
}

int OtpModule::Uart::writable()
{
    return Serial2.availableForWrite();
}

void OtpModule::Uart::write(uint8_t b)
{
    Serial2.write(b);
}

OtpModule::OtpModule() : MeshModule("Otp"), concurrency::OSThread("Otp"), link(uart, esp_random())
{
    if (inputBroker)
        inputObserver.observe(inputBroker);
}

int OtpModule::onInput(const InputEvent *event)
{
    if (!open)
        return 0;
    // 'x' injects a stray byte, 'b' drops the baud rate for a moment; any other key starts a new handshake
    if (event->kbchar == 'x')
        keyJunk = true;
    else if (event->kbchar == 'b')
        keyGlitch = true;
    else
        keyRestart = true;
    return 0;
}

int32_t OtpModule::runOnce()
{
    uint32_t now = millis();
    bool visible = lastDraw != 0 && Throttle::isWithinTimespanMs(lastDraw, 1000);

    if (!visible) {
        if (open) {
            Serial2.end();
            open = false;
            LOG_INFO("Otp: link closed");
        }
        return 200;
    }
    if (!open) {
        Serial2.setRxBufferSize(1024);
        Serial2.begin(OTP_BAUD, SERIAL_8N1, OTP_UART_RX, OTP_UART_TX);
        link.start(now);
        open = true;
        LOG_INFO("Otp: link open, UART2 %d baud, TX %d RX %d", OTP_BAUD, OTP_UART_TX, OTP_UART_RX);
    }

    if (keyRestart) {
        keyRestart = false;
        link.restart(now);
    }
    if (keyJunk) {
        keyJunk = false;
        Serial2.write(0x55);
    }
    if (keyGlitch) {
        keyGlitch = false;
        Serial2.updateBaudRate(9600);
        glitching = true;
        glitchAt = now;
    }
    if (glitching && Throttle::hasElapsed(glitchAt, 100)) {
        Serial2.updateBaudRate(OTP_BAUD);
        glitching = false;
    }

    link.poll(now);

    uint32_t rev = link.revision();
    if ((rev != lastRev && Throttle::hasElapsed(lastRedraw, 100)) || Throttle::hasElapsed(lastRedraw, otp::BEAT_MS)) {
        lastRev = rev;
        lastRedraw = now;
        UIFrameEvent e;
        e.action = UIFrameEvent::Action::REDRAW_ONLY;
        notifyObservers(&e);
    }
    return 1;
}

void OtpModule::drawHalf(OLEDDisplay *display, int16_t x, int16_t y, int rows, const char *name, const char *status,
                         const otp::Rows &r)
{
    char head[32];
    snprintf(head, sizeof(head), "%s  %s", name, status);
    display->drawString(x + 1, y, head);
    int lh = FONT_HEIGHT_SMALL;
    for (int i = 0; i < rows; i++) {
        int idx = (r.cur + 1 + otp::Rows::N - rows + i) % otp::Rows::N;
        display->drawString(x + 1, y + lh * (i + 1), r.text[idx]);
    }
}

void OtpModule::drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    lastDraw = millis();
    uint32_t now = lastDraw;
    display->setFont(FONT_SMALL);
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->setColor(WHITE);

    int half = display->getHeight() / 2;
    int rows = half / FONT_HEIGHT_SMALL - 1;
    if (rows > otp::Rows::N)
        rows = otp::Rows::N;

    otp::State s = link.state();
    const char *rxStatus = s == otp::LINKED ? "connected" : link.active(now) ? "Activity!" : "NC";

    char txStatus[24];
    static const char spin[] = "-\\|/";
    char beat = spin[(now / otp::BEAT_MS) % 4];
    switch (s) {
    case otp::BEAT:
        snprintf(txStatus, sizeof(txStatus), "beat %c", beat);
        break;
    case otp::REQ:
        snprintf(txStatus, sizeof(txStatus), "asking %c", beat);
        break;
    case otp::GEN:
        snprintf(txStatus, sizeof(txStatus), "generator %u%%", (unsigned)((uint64_t)link.done() * 100 / otp::BLOCK_BYTES));
        break;
    case otp::ACCEPT:
        snprintf(txStatus, sizeof(txStatus), "acceptor %u%%", (unsigned)((uint64_t)link.done() * 100 / otp::BLOCK_BYTES));
        break;
    case otp::LINKED:
        snprintf(txStatus, sizeof(txStatus), "beat %c", beat);
        break;
    default:
        snprintf(txStatus, sizeof(txStatus), "silent");
        break;
    }

    drawHalf(display, x, y, rows, "RX", rxStatus, link.rx);
    display->drawLine(x, y + half, x + display->getWidth(), y + half);
    drawHalf(display, x, y + half + 1, rows, "TX", txStatus, link.tx);
}

#endif
