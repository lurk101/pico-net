
#include "stdio_init.h"

#include "pico/stdio.h"
#if LIB_PICO_STDIO_UART
#include "pico/stdio_uart.h"
#endif
#if LIB_PICO_STDIO_USB
#include "pico/stdio_usb.h"
#include "tusb.h"
#endif

int stdio_init(void) {
    int r = 0;
#if LIB_PICO_STDIO_UART
    stdio_uart_init();
    getchar_timeout_us(1000);
    r |= STDIO_IS_UART;
#endif
#if LIB_PICO_STDIO_USB
    stdio_usb_init();
    while (!tud_cdc_connected())
        sleep_ms(1000);
    r |= STDIO_IS_USB;
#endif
    static const char* clear = "\033[H\033[J";
    for (const char* cp = clear; *cp; cp++)
        putchar_raw(*cp);
    return r;
}
