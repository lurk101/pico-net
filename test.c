/* Copyright (C) 1883 Thomas Edison - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GPLv2 license, which unfortunately won't be
 * written for another century.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "comm.h"
#include "stdinit.h"

#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

#define N 32

const char* test_msg = "Hello world ";

static void sputdec2(char* cp, uint32_t n, int first) {
    uint nl = n % 1000;
    uint nh = n / 1000;
    char buf[5];
    if (nh == 0) {
        sprintf(buf, "%d", nl);
        strcat(cp, buf);
    } else {
        sputdec2(cp, nh, 0);
        sprintf(buf, "%03d", nl);
        strcat(cp, buf);
    }
    if (!first)
        strcat(cp, ",");
}

static char* sputdec(uint32_t n) {
    static char buf[32];
    buf[0] = 0;
    sputdec2(buf, n, 1);
    return buf;
}

// application entry point
int main(void) {
    stdio_init();
    printf("starting\n");
    comm_init();
    comm_loop(false);
    printf("baud rate %s\n", sputdec(comm_baud()));
    int from;
    char buf[32];
    int count = 0;
    for (int i = N - 1; i > 0; i--) {
        strcpy(buf, test_msg);
        strcat(buf, sputdec((uint)-1 >> i));
        comm_transmit(comm_id(), buf, strlen(buf) + 1);
        i--;
        strcpy(buf, test_msg);
        strcat(buf, sputdec((uint)-1 >> i));
        comm_transmit(comm_id(), buf, strlen(buf) + 1);
        if (comm_receive_ready()) {
            comm_receive_blocking(&from, buf, sizeof(buf));
            count++;
            printf("%s from %u\n", buf, from);
        }
    }
    printf("tx done\n");
    while (count < N) {
        comm_receive_blocking(&from, buf, sizeof(buf));
        count++;
        printf("%s from %u\n", buf, from);
    }
    printf("done\n");
}
