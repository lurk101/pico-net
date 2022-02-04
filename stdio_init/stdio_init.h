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
