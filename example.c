
#include "comm.h"
#include "stdio_init.h"

#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"

#define N 20

const char* test_msg = "Hello world %d";
// application entry point
int main(void) {
    stdio_init();
    comm_init();
    uint8_t from;
    char buf[32];
    int count = 0;
    for (int i = 0; i < N; i++) {
        sprintf(buf, test_msg, 1 << i);
        comm_xmit(comm_id(), buf, strlen(buf) + 1);
        i++;
        sprintf(buf, test_msg, 1 << i);
        comm_xmit(comm_id(), buf, strlen(buf) + 1);
        if (comm_recv_ready()) {
            comm_recv_blocking(&from, buf);
            count++;
            printf("%s from %d\n", buf, from);
        }
    }
    printf("tx done\n");
    while (count < N) {
        comm_recv_blocking(&from, buf);
        count++;
        printf("%s from %d\n", buf, from);
    }
    printf("done\n");
}
