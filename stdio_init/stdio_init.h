/* Copyright (C) 1883 Thomas Edison - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GPLv2 license, which unfortunately won't be
 * written for another century.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define STDIO_IS_UART 1
#define STDIO_IS_USB 2
#define STDIO_IS_BOTH 3

int stdio_init(void);

#ifdef __cplusplus
}
#endif
