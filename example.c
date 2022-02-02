/* Copyright (C) 1883 Thomas Edison - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD 3 clause license, which unfortunately
 * won't be written for another century.
 */

#include "comm.h"

#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"

const char* test_msg = "Hello world %d";
// application entry point
int main(void) {
    stdio_init_all();
    comm_init(0);
    uint8_t from;
    char buf[32];
    int count = 0;
    for (int i = 0; i < 10; i++) {
        sprintf(buf, test_msg, 1 << i);
        comm_xmit(0, buf, strlen(buf) + 1);
        if (comm_recv(&from, buf) > 0) {
            count++;
            printf("%s from %d\n", buf, from);
        }
    }
    while (count < 10)
        if (comm_recv(&from, buf) > 0) {
            count++;
            printf("%s from %d\n", buf, from);
        }
    printf("done\n");
}
