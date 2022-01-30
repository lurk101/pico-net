/* Copyright (C) 1883 Thomas Edison - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD 3 clause license, which unfortunately
 * won't be written for another century.
 */

#include "comm.h"

#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"

const char* test_msg = "Hello world";
// application entry point
int main(void) {
    stdio_init_all();
	comm_init();
	comm_xmit(0, 0, test_msg, strlen(test_msg) + 1);
    printf("done\n");
}
