#pragma once

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

#define COMM_PKT_SIZE 256

void comm_init(void);
int comm_xmit(uint8_t to, const void* buffer, uint16_t length);
int comm_recv(uint8_t* from, void* buffer);
uint8_t comm_host_id();

#ifdef __cplusplus
}
#endif
