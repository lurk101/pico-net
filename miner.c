/* Copyright (C) 1883 Thomas Edison - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GPLv2 license, which unfortunately won't be
 * written for another century.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "comm.h"
#include "sha256.h"

#include "hardware/irq.h"
#include "hardware/pwm.h"

#include "pico/stdlib.h"

#include <stdio.h>
#include <string.h>

#define LED_GPIO PICO_DEFAULT_LED_PIN
//#define LED_GPIO 22 // custom LED

enum { start_msg_id = 0, stop_msg_id, solution_msg_id };

typedef union {
    struct {
        int8_t msg_id;
        char hdr[512 / 8];
        uint32_t bits;
        uint32_t nonce;
    } start_msg;
    struct {
        int8_t msg_id;
        uint32_t start;
        float t;
    } solution_msg;
    struct {
        int8_t msg_id;
    } stop_msg;
} msg_t;

static int master;
static uint32_t target;
static uint32_t nonce_core0, nonce_core1;
static uint32_t mine = 0;

static char line[128];
static uint32_t start_time;
static char header[512 / 8];

static uint8_t fade = 0;

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
        m.start_msg.msg_id = start_msg_id;
        memcpy(m.start_msg.hdr, header, sizeof(header));
        m.start_msg.bits = targ_to_bits(target);
        uint32_t delta = (1ull << 32) / COMM_NODES;
        for (int n = 0; n < COMM_NODES; n++) {
            m.start_msg.nonce = n * delta;
            comm_transmit(n, &m, sizeof(m.start_msg));
        }
    } else if (strcmp(cp1, "stop") == 0) {
        m.stop_msg.msg_id = stop_msg_id;
        comm_transmit(COMM_NODES, &m, sizeof(m.stop_msg));
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
    switch (m.stop_msg.msg_id) {
    case start_msg_id:
        start_time = time_us_32();
        memcpy(header, m.start_msg.hdr, sizeof(header));
        target = bits_to_targ(m.start_msg.bits);
        nonce_core0 = m.start_msg.nonce;
        nonce_core1 = nonce_core0 + 1;
        master = from;
        printf("Header: %s\nTarget: %08x\nStart:  %08x from node %d\nNode %d searching\n", header,
               target, nonce_core0, from, comm_id());
        mine = 1;
        break;
    case solution_msg_id:
        printf("Solution %08x from node %d at %.2f\n", m.solution_msg.start, from,
               m.solution_msg.t);
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
        m.solution_msg.msg_id = solution_msg_id;
        m.solution_msg.start = s;
        m.solution_msg.t = (time_us_32() - start_time) / 1e6;
        if (comm_id() != master)
            printf("Found %08x\n", s);
        fade = 255;
        comm_transmit(master, &m, sizeof(m.solution_msg));
    }
}

void core1_idle(void) {
    if (mine) {
        check_hash(nonce_core1);
        nonce_core1 += 2;
    }
}

void on_pwm_wrap() {
    static bool going_up = true;
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(LED_GPIO));
    if (fade)
        fade--;
    // Square the fade value to make the LED's brightness appear more linear
    // Note this range matches with the wrap value
    pwm_set_gpio_level(LED_GPIO, (uint16_t)fade * fade);
}

// application entry point
int main(void) {
    stdio_init_all();
    comm_init(core1_idle, NULL);
    // Tell the LED pin that the PWM is in charge of its value.
    gpio_set_function(LED_GPIO, GPIO_FUNC_PWM);
    // Figure out which slice we just connected to the LED pin
    uint slice_num = pwm_gpio_to_slice_num(LED_GPIO);
    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Get some sensible defaults for the slice configuration. By default, the
    // counter is allowed to wrap over its maximum range (0 to 2**16-1)
    pwm_config config = pwm_get_default_config();
    // Set divider, reduces counter clock to sysclock/this value
    pwm_config_set_clkdiv(&config, 4);
    // Load the configuration into our PWM slice, and set it running.
    pwm_init(slice_num, &config, true);

    // Everything after this point happens in the PWM interrupt handler, so we
    // can twiddle our thumbs

    printf("Started Bitcoin miner node %d\nLink speed: %u baud\n", comm_id(), comm_baud());
    for (;;) {
        check_console();
        check_messages();
        if (mine) {
            check_hash(nonce_core0);
            nonce_core0 += 2;
        }
    }
}
