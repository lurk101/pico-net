/* Copyright (C) 1883 Thomas Edison - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GPLv2 license, which unfortunately won't be
 * written for another century.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "stdinit.h"

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
    stdio_uart_init();        // setup uart 0
    getchar_timeout_us(1000); // discard initial garbage character
    r |= STDIO_IS_UART;
#endif

#if LIB_PICO_STDIO_USB
    stdio_usb_init();            // setup usb endpoint
    while (!tud_cdc_connected()) // wait for actual connection
        sleep_ms(1000);
    r |= STDIO_IS_USB;
#endif
    // clear the screen on VT terminal
    static const char* clear = "\033[H\033[J";
    for (const char* cp = clear; *cp; cp++)
        putchar_raw(*cp);
    return r;
}
