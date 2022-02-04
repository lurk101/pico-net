
#include "comm.h"
#include "stdio_init.h"

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
    printf("baud rate %s\n", sputdec(comm_baud()));
    int from;
    char buf[32];
    int count = 0;
    for (int i = 0; i < N; i++) {
        strcpy(buf, test_msg);
        strcat(buf, sputdec(1 << i));
        comm_xmit(comm_id(), buf, strlen(buf) + 1);
        i++;
        strcpy(buf, test_msg);
        strcat(buf, sputdec(1 << i));
        comm_xmit(comm_id(), buf, strlen(buf) + 1);
        if (comm_recv_ready()) {
            comm_recv_blocking(&from, buf, sizeof(buf));
            count++;
            printf("%s from %u\n", buf, from);
        }
    }
    printf("tx done\n");
    while (count < N) {
        comm_recv_blocking(&from, buf, sizeof(buf));
        count++;
        printf("%s from %u\n", buf, from);
    }
    printf("done\n");
}
