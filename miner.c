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

enum { start_msg_id = 0, stop_msg_id, solution_msg_id };

typedef struct {
    uint32_t msg_id;
    union {
        struct {
            char hdr[512 / 8];
            uint32_t bits;
            uint32_t nonce;
        } start_msg;
        struct {
            uint32_t start;
        } solution_msg;
        struct {
        } stop_msg;
    } msgs;
} msg_t;

static int master;
static uint32_t target;
static uint32_t nonce_core0, nonce_core1;
static uint32_t mine = 0;

static char line[128];
static uint32_t start_time;
static char header[512 / 8];

static char* skip_blanks(char* cp) {
    while (*cp && ((*cp == ' ') || (*cp == '\t')))
        cp++;
    return cp;
}

static char* skip_to_blank(char* cp) {
    while (*cp && (*cp != ' ') && (*cp != '\t'))
        cp++;
    return cp;
}

uint32_t targ_to_bits(uint32_t t) {
    uint32_t l = 32;
    while (t && ((t & 0xff000000) == 0)) {
        l--;
        t <<= 8;
    }
    return (t >> 8) | (l << 24);
}

uint32_t bits_to_targ(uint32_t b) {
    uint32_t l = b >> 24;
    b = (b & 0xffffff) << 8;
    while (l < 32) {
        l++;
        b >>= 8;
    }
    return b;
}

static void process_command(void) {
    msg_t m;
    char* cp1 = skip_blanks(line);
    char* cp2 = skip_to_blank(cp1);
    char* cp3 = skip_blanks(cp2);
    *cp2 = 0;
    if (strcmp(cp1, "head") == 0) {
        if (*cp3 == 0)
            printf("Need to specify block content\n");
        else {
            printf("header is '%s' padded with zeros to 64 bytes\n", cp3);
            memset(header, 0, sizeof(header));
            strcpy(header, cp3);
        }
    } else if (strcmp(cp1, "targ") == 0) {
        if (*cp3 == 0)
            printf("Need to specify a target value\n");
        else {
            sscanf(cp3, "%u", &target);
            printf("Target is: %08x\n", target);
        }
    } else if (strcmp(cp1, "go") == 0) {
        m.msg_id = start_msg_id;
        memcpy(m.msgs.start_msg.hdr, header, sizeof(header));
        m.msgs.start_msg.bits = targ_to_bits(target);
        uint32_t delta = (1ull << 32) / COMM_NODES;
        for (int n = 0; n < COMM_NODES; n++) {
            m.msgs.start_msg.nonce = n * delta;
            comm_transmit(n, &m, sizeof(m.msgs.start_msg) + 4);
        }
    } else if (strcmp(cp1, "stop") == 0) {
        m.msg_id = stop_msg_id;
        comm_transmit(COMM_NODES, &m, sizeof(m.msgs.stop_msg) + 4);
    } else
        printf("Unknown command '%s'\n", cp1);
}

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
        if (lp > line) {
            lp--;
            putchar('\b');
            putchar(' ');
            putchar('\b');
        }
        break;
    default:
        if ((c < ' ') || (c >= 0x7f))
            break;
        putchar(c);
        *lp++ = c;
    }
}

static void check_messages(void) {
    if (!comm_receive_ready())
        return;
    msg_t m;
    int from;
    comm_receive_blocking(&from, &m, sizeof(m));
    switch (m.msg_id) {
    case start_msg_id:
        start_time = time_us_32();
        memcpy(header, m.msgs.start_msg.hdr, sizeof(header));
        target = bits_to_targ(m.msgs.start_msg.bits);
        nonce_core0 = m.msgs.start_msg.nonce;
        nonce_core1 = nonce_core0 + 1;
        master = from;
        printf("Header: %s\nTarget: %08x\nStart:  %08x from node %d\nNode %d searching\n", header,
               target, nonce_core0, from, comm_id());
        mine = 1;
        break;
    case solution_msg_id:
        printf("Solution %08x from node %d at %.2f\n", m.msgs.solution_msg.start, from,
               (time_us_32() - start_time) / 1e6);
        break;
    case stop_msg_id:
        mine = 0;
        printf("Node %d stopped\n", comm_id());
        break;
    }
}

void check_hash(uint32_t s) {
    sha256_t s256;
    sha256_init(&s256);
    sha256_update(&s256, header, sizeof(header));
    sha256_update(&s256, &s, sizeof(s));
    uint dgst[8];
    sha256_digest(&s256, &dgst);
    sha256_init(&s256);
    sha256_update(&s256, &dgst, sizeof(dgst));
    sha256_digest(&s256, &dgst);
    if (__builtin_bswap32(dgst[0]) <= target) {
        msg_t m;
        m.msg_id = solution_msg_id;
        m.msgs.solution_msg.start = s;
        if (comm_id() != master)
            printf("Found %08x\n", s);
        comm_transmit(master, &m, sizeof(m.msgs.solution_msg) + 4);
    }
}

void core1_idle(void) {
    if (mine) {
        check_hash(nonce_core1);
        nonce_core1 += 2;
    }
}

// application entry point
int main(void) {
    stdio_init();
    comm_init(core1_idle);
    printf("Starting Bitcoin miner node %d\nLink speed: %u baud\n", comm_id(), comm_baud());
    for (;;) {
        check_console();
        check_messages();
        if (mine) {
            check_hash(nonce_core0);
            nonce_core0 += 2;
        }
    }
}
