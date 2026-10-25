#include "Otp.h"
#include "OtpModule.h"

void setupOtp()
{
#if defined(OTP_APP) && defined(OTP_UART_TX) && HAS_SCREEN
    new OtpModule();
#endif
}
