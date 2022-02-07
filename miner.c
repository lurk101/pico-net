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

#define NODES 1

typedef uint32_t digest_t[8];

enum { start_msg_id = 1, stop_msg_id, solution_msg_id };

typedef struct {
    uint32_t msg_id;
    union {
        struct {
            digest_t dgst;
            uint32_t diff;
            uint32_t start;
        } start_msg;
        struct {
            uint32_t start;
        } solution_msg;
    } msgs;
} msg_t;

static int master;
static digest_t digest;
static uint32_t diff;
static uint32_t start0, start1;
static uint32_t mine = 0;

static char line[128];
static uint32_t start_time;

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

static void dump(void* buf, uint32_t len) {
    uint8_t* cp = (char*)buf;
    while (len--)
        printf("%02x", *cp++);
}

static void process_command(void) {
    msg_t m;
    char* cp1 = skip_blanks(line);
    char* cp2 = skip_to_blank(cp1);
    char* cp3 = skip_blanks(cp2);
    char* cp4 = skip_to_blank(cp3);
    *cp2 = 0;
    *cp4 = 0;
    if (strcmp(cp1, "block") == 0) {
        if (*cp3 == 0)
            printf("Need to specify block content\n");
        else {
            printf("SHA256 hash of '%s' is: ", cp3);
            sha256_t s256;
            sha256_init(&s256);
            sha256_update(&s256, cp3, strlen(cp3));
            sha256_digest(&s256, &digest);
            dump(&digest, sizeof(digest_t));
            printf("\n");
        }
    } else if (strcmp(cp1, "dif") == 0) {
        if (*cp3 == 0)
            printf("Need to specify a difficulty\n");
        else {
            sscanf(cp3, "%u", &diff);
            printf("Difficulty is: %08x\n", diff);
        }
    } else if (strcmp(cp1, "start") == 0) {
        m.msg_id = start_msg_id;
        memcpy(m.msgs.start_msg.dgst, digest, sizeof(digest));
        m.msgs.start_msg.diff = diff;
        for (int n = 0; n < NODES; n++) {
            m.msgs.start_msg.start = n;
            comm_transmit(n, &m, sizeof(m.msgs.start_msg) + 4);
        }
    } else if (strcmp(cp1, "stop") == 0) {
        m.msg_id = stop_msg_id;
        for (int n = 0; n < NODES; n++)
            comm_transmit(n, &m, 4);
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
        memcpy(digest, m.msgs.start_msg.dgst, sizeof(digest));
        diff = m.msgs.start_msg.diff;
        start0 = m.msgs.start_msg.start;
        start1 = start0 + 1;
        master = from;
        printf("Digest: ");
        dump(&digest, sizeof(digest));
        printf("\nDiff: %08x\nStart: %08x from node %d\n", diff, start0, from);
        mine = 1;
        break;
    case solution_msg_id:
        printf("Got solution %08x from node %d\n", m.msgs.solution_msg.start, from);
        printf("Hash rate: %u H/s\n",
               (uint32_t)(m.msgs.solution_msg.start * 1e6 / (time_us_32() - start_time)));
        break;
    case stop_msg_id:
        mine = 0;
        printf("Node %d stopped\n", comm_id());
        break;
    default:;
    }
}

void check_hash(uint32_t s) {
    sha256_t s256;
    sha256_init(&s256);
    sha256_update(&s256, &digest, sizeof(digest));
    sha256_update(&s256, &s, sizeof(s));
    digest_t dgst;
    sha256_digest(&s256, &dgst);
    sha256_init(&s256);
    sha256_update(&s256, &dgst, sizeof(dgst));
    sha256_digest(&s256, &dgst);
    if (__builtin_bswap32(dgst[0]) <= diff) {
        msg_t m;
        m.msg_id = solution_msg_id;
        m.msgs.solution_msg.start = s;
        comm_transmit(master, &m, sizeof(m.msgs.solution_msg) + 4);
    }
}

void core1_idle(void) {
    if (mine) {
        check_hash(start1);
        start1 += NODES * 2;
    }
}

// application entry point
int main(void) {
    stdio_init();
    printf("starting\n");
    comm_init(core1_idle);
    comm_loop(true);
    for (;;) {
        check_console();
        check_messages();
        if (mine) {
            check_hash(start0);
            start0 += NODES * 2;
        }
    }
    printf("done\n");
}
