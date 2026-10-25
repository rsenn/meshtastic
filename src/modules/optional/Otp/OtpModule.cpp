#include "OtpModule.h"

#if defined(OTP_APP) && defined(OTP_UART_TX) && HAS_SCREEN

#include "esp_random.h"
#include "graphics/Screen.h"
#include "graphics/ScreenFonts.h"
#include "main.h"
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

void OtpModule::Uart::setBaud(uint32_t baud)
{
    Serial2.updateBaudRate(baud);
}

void OtpModule::Uart::flush()
{
    // returns when the TX FIFO is empty, not only the ring
    Serial2.flush();
}

int OtpModule::Uart::available()
{
    return Serial2.available();
}

uint32_t OtpModule::Uart::us()
{
    return micros();
}

void OtpModule::Sink::line(const char *s)
{
    LOG_INFO("Otp: %s", s);
}

OtpModule::OtpModule() : MeshModule("Otp"), concurrency::OSThread("Otp"), link(uart, esp_random()), suite(link, &sink)
{
    link.setLogger(&sink);
    if (inputBroker)
        inputObserver.observe(inputBroker);

    // core 0 with WiFi and BLE, above the Arduino loop (priority 1), so the LCD flush on core 1 cannot delay it
    xTaskCreatePinnedToCore(taskMain, "OtpLink", 8192 / sizeof(StackType_t), this, 5, nullptr, 0);
}

void OtpModule::taskMain(void *self)
{
    static_cast<OtpModule *>(self)->taskLoop();
}

// protocol side, every millisecond: link and suite do all the work while the UART is claimed
void OtpModule::taskLoop()
{
    for (;;) {
        {
            concurrency::LockGuard guard(&lock);
            if (open) {
                uint32_t now = millis();
                if (keySpace) {
                    keySpace = false;
                    suite.press(now);
                }
                link.poll(now);
                suite.poll(now);
            }
        }
        vTaskDelay(1);
    }
}

// While our frame is on screen the keys are ours. Without this Screen treats a space as "next frame" (Screen.cpp, ANYKEY ' ')
// and carried the user off the test screen on every press. Interception stops 3 s after the last draw.
bool OtpModule::interceptingKeyboardInput()
{
    return open && lastDraw != 0 && Throttle::isWithinTimespanMs(lastDraw, 3000);
}

int OtpModule::onInput(const InputEvent *event)
{
    if (!interceptingKeyboardInput())
        return 0;

    // space is the only test key: next test group once the link is idle
    if (event->inputEvent == INPUT_BROKER_ANYKEY && event->kbchar == ' ') {
        keySpace = true;
        return 0;
    }

    // Screen ignores navigation while a module intercepts, so the way out is ours: left and back go to the previous frame,
    // right and press to the next, long up and down likewise. lastDraw reset ends the interception at once.
    bool prev = event->inputEvent == INPUT_BROKER_LEFT || event->inputEvent == INPUT_BROKER_BACK ||
                event->inputEvent == INPUT_BROKER_UP_LONG;
    bool next = event->inputEvent == INPUT_BROKER_RIGHT || event->inputEvent == INPUT_BROKER_USER_PRESS ||
                event->inputEvent == INPUT_BROKER_DOWN_LONG;
    if ((prev || next) && screen) {
        lastDraw = 0;
        if (prev)
            screen->showPrevFrame();
        else
            screen->showNextFrame();
    }
    return 0;
}

int32_t OtpModule::runOnce()
{
    uint32_t now = millis();
    // The frame counts as shown for 4 s after its last draw (quiet blocks redraw once a second), and a run in progress is never
    // closed under its feet: the first hardware run with quiet blocks closed the link a second into every block.
    bool busy = suite.running() || (link.state() != otp::LINKED && link.state() != otp::BEAT && link.state() != otp::SLEEP);
    bool visible = busy || (lastDraw != 0 && Throttle::isWithinTimespanMs(lastDraw, 4000));

    // claim or release the UART with the link task held off
    {
        concurrency::LockGuard guard(&lock);
        if (!visible && open) {
            Serial2.end();
            open = false;
            LOG_INFO("Otp: link closed");
        } else if (visible && !open) {
            // ring sizes must be set before begin(); the RX ring has to hold a whole window of unechoed bytes
            Serial2.setRxBufferSize(OTP_RX_RING);
            Serial2.setTxBufferSize(OTP_TX_RING);
            Serial2.onReceiveError([this](hardwareSerial_error_t e) {
                switch (e) {
                case UART_FIFO_OVF_ERROR:
                    uart.errs.fifo++;
                    break;
                case UART_BUFFER_FULL_ERROR:
                    uart.errs.ring++;
                    break;
                case UART_FRAME_ERROR:
                    uart.errs.frame++;
                    break;
                case UART_PARITY_ERROR:
                    uart.errs.parity++;
                    break;
                case UART_BREAK_ERROR:
                    uart.errs.brk++;
                    break;
                default:
                    break;
                }
            });
            Serial2.begin(OTP_BAUD, SERIAL_8N1, OTP_UART_RX, OTP_UART_TX);
            link.start(now);
            open = true;
            LOG_INFO("Otp: link open, UART2 %d baud, TX %d RX %d, rings %d/%d", OTP_BAUD, OTP_UART_TX, OTP_UART_RX, OTP_RX_RING,
                     OTP_TX_RING);
        }
    }
    if (!visible)
        return 200;

    // screen side: redraw on change, but only once a second while a quiet block runs, the LCD flush stalls this thread
    bool data = link.state() == otp::GEN || link.state() == otp::ACCEPT;
    bool quiet = data && !link.run().dump;
    uint32_t minGap = quiet ? 1000 : 100;
    uint32_t rev = link.revision();
    if ((rev != lastRev && Throttle::hasElapsed(lastRedraw, minGap)) || Throttle::hasElapsed(lastRedraw, quiet ? 1000 : otp::BEAT_MS)) {
        lastRev = rev;
        lastRedraw = now;
        UIFrameEvent e;
        e.action = UIFrameEvent::Action::REDRAW_ONLY;
        notifyObservers(&e);
    }
    return 20;
}

void OtpModule::drawHalf(OLEDDisplay *display, int16_t x, int16_t y, int rows, const char *name, const char *status,
                         const char *tag, const char *const *lines)
{
    char head[32];
    snprintf(head, sizeof(head), "%s  %s", name, status);
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->drawString(x + 1, y, head);

    // run mode tag at the right edge of the header
    if (tag[0]) {
        display->setTextAlignment(TEXT_ALIGN_RIGHT);
        display->drawString(x + display->getWidth(), y, tag);
        display->setTextAlignment(TEXT_ALIGN_LEFT);
    }

    int lh = FONT_HEIGHT_SMALL;
    for (int i = 0; i < rows; i++)
        display->drawString(x + 1, y + lh * (i + 1), lines[i]);
}

void OtpModule::drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    lastDraw = millis();

    // the link task rewrites rows and state at any time; read them as one snapshot
    concurrency::LockGuard guard(&lock);
    uint32_t now = lastDraw;
    display->setFont(FONT_SMALL);
    display->setColor(WHITE);

    int half = display->getHeight() / 2;
    int rows = half / FONT_HEIGHT_SMALL - 1;
    if (rows > otp::Rows::N)
        rows = otp::Rows::N;

    otp::State s = link.state();
    bool data = s == otp::GEN || s == otp::ACCEPT;
    bool dump = link.run().dump;
    unsigned pct = link.total() ? (unsigned)((uint64_t)link.done() * 100 / link.total()) : 0;

    // headers: RX says what the line does, TX what we do
    char rxStatus[24];
    if (s == otp::LINKED)
        snprintf(rxStatus, sizeof(rxStatus), "connected");
    else if (data)
        snprintf(rxStatus, sizeof(rxStatus), "%s %u%%", s == otp::GEN ? "generator" : "acceptor", pct);
    else
        snprintf(rxStatus, sizeof(rxStatus), "%s", link.active(now) ? "Activity!" : "NC");

    char txStatus[24];
    static const char spin[] = "-\\|/";
    char beat = spin[(now / otp::BEAT_MS) % 4];
    switch (s) {
    case otp::BEAT:
    case otp::LINKED:
        snprintf(txStatus, sizeof(txStatus), "beat %c", beat);
        break;
    case otp::REQ:
        snprintf(txStatus, sizeof(txStatus), "asking %c", beat);
        break;
    case otp::NEGO:
    case otp::SWITCH:
        snprintf(txStatus, sizeof(txStatus), "speed %c", beat);
        break;
    case otp::GEN:
        snprintf(txStatus, sizeof(txStatus), "generator %u%%", pct);
        break;
    case otp::ACCEPT:
        snprintf(txStatus, sizeof(txStatus), "acceptor %u%%", pct);
        break;
    default:
        snprintf(txStatus, sizeof(txStatus), "silent");
        break;
    }

    // run mode tag: "115k dump", "2M quiet"
    char tag[16];
    uint32_t baud = link.run().baud;
    if (baud >= 1000000)
        snprintf(tag, sizeof(tag), "%uM %s", (unsigned)(baud / 1000000), dump ? "dump" : "quiet");
    else
        snprintf(tag, sizeof(tag), "%uk %s", (unsigned)(baud / 1000), dump ? "dump" : "quiet");

    // rows: wiring help until the first byte, progress numbers in quiet blocks, else the link's own log rows
    char rxBuf[otp::Rows::N][otp::Rows::W] = {};
    char txBuf[otp::Rows::N][otp::Rows::W] = {};
    const char *rxLine[otp::Rows::N];
    const char *txLine[otp::Rows::N];
    for (int i = 0; i < rows; i++) {
        int idx = (link.rx.cur + 1 + otp::Rows::N - rows + i) % otp::Rows::N;
        rxLine[i] = link.rx.text[idx];
        idx = (link.tx.cur + 1 + otp::Rows::N - rows + i) % otp::Rows::N;
        txLine[i] = link.tx.text[idx];
    }

    if (!link.everReceived()) {
        // wiring help; a correct link reacts within ~200 ms, so silence after wiring means the wiring is wrong
        snprintf(rxBuf[0], otp::Rows::W, "%s", OTP_PIN_HINT1);
        snprintf(rxBuf[1], otp::Rows::W, "%s", OTP_PIN_HINT2);
        snprintf(txBuf[0], otp::Rows::W, "%s", OTP_WIRED_HINT1);
        snprintf(txBuf[1], otp::Rows::W, "%s", OTP_WIRED_HINT2);
        for (int i = 0; i < rows; i++) {
            rxLine[i] = rxBuf[i];
            txLine[i] = txBuf[i];
        }
    } else {
        if (data && !dump) {
            uint32_t ms = link.runMs(now);
            snprintf(rxBuf[0], otp::Rows::W, "%uk of %uk  %u kB/s", (unsigned)(link.done() / 1024), (unsigned)(link.total() / 1024),
                     ms ? (unsigned)((uint64_t)link.done() / ms) : 0);
            snprintf(rxBuf[1], otp::Rows::W, "%u baud", (unsigned)link.baud());
            for (int i = 0; i < rows; i++)
                rxLine[i] = rxBuf[i];
        }

        // idle or testing: TX half names what space does next and the newest case outcome
        if (s == otp::LINKED || suite.running()) {
            if (suite.running())
                snprintf(txBuf[0], otp::Rows::W, "%s %u/%u %s", otp::Suite::name(suite.group()), suite.step() + 1, suite.steps(),
                         suite.current());
            else
                snprintf(txBuf[0], otp::Rows::W, "space: %s test", otp::Suite::name(suite.next()));
            snprintf(txBuf[1], otp::Rows::W, "%s", suite.last());
            for (int i = 0; i < rows; i++)
                txLine[i] = txBuf[i];
        }
    }

    drawHalf(display, x, y, rows, "RX", rxStatus, "", rxLine);
    display->drawLine(x, y + half, x + display->getWidth(), y + half);
    drawHalf(display, x, y + half + 1, rows, "TX", txStatus, tag, txLine);
}

#endif
