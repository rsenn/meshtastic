#pragma once

#include "OtpConfig.h"
#include "configuration.h"

#if defined(OTP_APP) && defined(OTP_UART_TX) && HAS_SCREEN

#include "Observer.h"
#include "OtpLink.h"
#include "concurrency/OSThread.h"
#include "input/InputBroker.h"
#include "mesh/MeshModule.h"

// Serial link test screen: RX monitor on the upper half, TX monitor on the lower half.
class OtpModule : public MeshModule, public Observable<const UIFrameEvent *>, private concurrency::OSThread
{
  public:
    OtpModule();

    void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y) override;

  protected:
    int32_t runOnce() override;
    bool wantUIFrame() override { return true; }
    bool wantPacket(const meshtastic_MeshPacket *) override { return false; }
    Observable<const UIFrameEvent *> *getUIFrameObservable() override { return this; }

  private:
    struct Uart : otp::Port {
        int read() override;
        int writable() override;
        void write(uint8_t b) override;
    };

    int onInput(const InputEvent *event);
    void drawHalf(OLEDDisplay *display, int16_t x, int16_t y, int rows, const char *name, const char *status,
                  const otp::Rows &r);

    CallbackObserver<OtpModule, const InputEvent *> inputObserver =
        CallbackObserver<OtpModule, const InputEvent *>(this, &OtpModule::onInput);
    Uart uart;
    otp::Link link;
    bool open = false;
    uint32_t lastDraw = 0, lastRedraw = 0, lastRev = 0, glitchAt = 0;
    volatile bool keyRestart = false, keyJunk = false, keyGlitch = false;
    bool glitching = false;
};

#endif
