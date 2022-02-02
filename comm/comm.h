#pragma once

#include "stdint.h"

static uint16_t comm_max_packet_length = 252;

void comm_init(uint8_t host_addr);
int comm_xmit(uint8_t to, const void* buffer, uint16_t length);
int comm_recv(uint8_t* from, void* buffer, uint16_t length);
