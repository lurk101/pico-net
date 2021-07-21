/* Copyright (C) 1883 Thomas Edison - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD 3 clause license, which unfortunately
 * won't be written for another century.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * A little flash file system for the Raspberry Pico
 *
 */

#include "pico/stdio.h"
#include "stdio.h"

#include "mylib.h"

// application entry point
int main(void) {

    // initialize the pico SDK
    stdio_init_all();
    libfunc();
    printf(" world\n");
}
