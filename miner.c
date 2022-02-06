/* Copyright (C) 1883 Thomas Edison - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GPLv2 license, which unfortunately won't be
 * written for another century.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "comm.h"
#include "sha256.h"
#include "stdinit.h"

#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

#define N 32

typedef union {
    uint8_t b[256 / sizeof(uint8_t)];
    uint64_t w[256 / sizeof(uint64_t)];
} digest_u;

static char line[128];

static void process_command(void) { printf("%s\n", line); }

static void check_console(void) {
    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT)
        return;
    static char* lp = line;
    switch (c) {
    case '\r': // CR
    case '\n': // LF
        putchar('\r');
        putchar('\n');
        *lp = '\0';
        process_command();
        lp = line;
        break;
    case '\b':   // BS
    case '\x7F': // DEL
        if (lp > line) {
            lp--;
            putchar('\b');
            putchar(' ');
            putchar('\b');
        }
        break;
    default:
        if (c < ' ')
            break;
        putchar(c);
        *lp++ = c;
    }
}

// application entry point
int main(void) {
    stdio_init();
    printf("starting\n");
    comm_init();
    comm_loop(false);
    for (;;) {
        check_console();
    }
    printf("done\n");
}
