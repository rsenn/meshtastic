#pragma once

#include "OtpConfig.h"
#include "configuration.h"

#if defined(OTP_APP) && defined(OTP_UART_TX) && HAS_SCREEN

#include "Observer.h"
#include "OtpLink.h"
#include "OtpSuite.h"
#include "concurrency/Lock.h"
#include "concurrency/LockGuard.h"
#include "concurrency/OSThread.h"
#include "input/InputBroker.h"
#include "mesh/MeshModule.h"

// Serial link test screen: RX monitor on the upper half, TX monitor on the lower half.
// Binds the pure link and suite (OtpLink, OtpSuite) to Serial2, the keys, the screen and the host log.
// The link runs in a FreeRTOS task of its own: the LCD flush and the other module threads stall the main loop for 100+ ms,
// which broke the +-50% heartbeat window on the first hardware run (design-docs OTP_APP_DESIGN.md 15.10).
class OtpModule : public MeshModule, public Observable<const UIFrameEvent *>, private concurrency::OSThread
{
  public:
    OtpModule();

    void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y) override;

  protected:
    int32_t runOnce() override;
    bool wantUIFrame() override { return true; }
    bool interceptingKeyboardInput() override;
    bool wantPacket(const meshtastic_MeshPacket *) override { return false; }
    Observable<const UIFrameEvent *> *getUIFrameObservable() override { return this; }

  private:
    // the UART as the link sees it
    struct Uart : otp::Port {
        int read() override;
        int writable() override;
        void write(uint8_t b) override;
        void setBaud(uint32_t baud) override;
        void flush() override;
        int available() override;
        uint32_t us() override;
        otp::Errs errors() override { return errs; }

        otp::Errs errs; // counted by the driver's event task through onReceiveError
    };

    // link and suite lines go to the USB console
    struct Sink : otp::Logger {
        void line(const char *s) override;
    };

    int onInput(const InputEvent *event);
    static void taskMain(void *self);
    void taskLoop();
    void drawHalf(OLEDDisplay *display, int16_t x, int16_t y, int rows, const char *name, const char *status, const char *tag,
                  const char *const *lines);

    CallbackObserver<OtpModule, const InputEvent *> inputObserver =
        CallbackObserver<OtpModule, const InputEvent *>(this, &OtpModule::onInput);
    Uart uart;
    Sink sink;
    otp::Link link;
    otp::Suite suite;
    concurrency::Lock lock;                     // link, suite and UART state: held by the link task, runOnce and drawFrame
    volatile bool open = false;                 // UART claimed, only while the frame is on screen
    uint32_t lastDraw = 0;                      // drawFrame ran, ms
    uint32_t lastRedraw = 0, lastRev = 0;       // last redraw we asked for and the link revision it showed
    volatile bool keySpace = false;             // set by the input observer, consumed in runOnce
};

#endif
