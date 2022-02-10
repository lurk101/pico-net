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

// Pico network protocol stack, physical, data link and network layers.
//
// Physical layer
//
// Standard 3.3 volt GPIO transmit and receive. Single transmit pin
// and receive pin with single wire connection. All devices share common
// ground.
//
// Data link layer.
//
// 32n1 UART implemented on PIO. Packet data is serialized into 1 start bit,
// 32 data bits and no parity bit. The transmitter rounds messages to 32
// bit word size and prepends the message size in words and the from and to
// addresses for transmission. Receiver expects to receive that many words.
// No error detection or correction is applied. The link is expected to be
// perfect.
//
// Network layer
//
// Packet routing is accomplished using a simple 1 byte node id. The nodes
// are organized in a unidirectional ring, and each node acts as an add-drop
// multiplexer. A packet received with a destination address matching the
// node's id it is queued on the nodes received packet queue for application
// retrieval. If the packet is not for his node it is transmitted to the next
// node in the ring.

#define COMM_PKT_SIZE 1024 // Maximum packet size
#define COMM_NODES 4       // Nodes in ring and broadcast address

// Initialize. Set up the 32 bit PIO UART, 3 DMA channels,
// and buffer queues.
// Parameters:
//    idle - function pointer to core1 idle handler, NULL = none
// Return: none
//
void comm_init(void (*idle)(void));

// Send a packet.
// Parameters:
//    to - destination node id
//    buffer - pointer to packet data
//    length - length of packet
// Return: none.
//
void comm_transmit(int to, const void* buffer, int length);

// receive packet
// Parameters:
//    from - pointer to packet's source node id
//    buffer - pointer to packet data
//    length - length of packet buffer
// Return: length of received packet
//
int comm_receive_blocking(int* from, void* buffer, int buf_size);

// Packet available ?
// Parameters: none
// Return: 0 is not ready, otherwise packet(s) available
//
int comm_receive_ready(void);

// Get node id
// Parameters: none
// Return: this node's id
//
int comm_id(void);

// Get effective baud rate
// Parameters: none
// Return: UART link's bit rate
//
int comm_baud(void);

#ifdef __cplusplus
}
#endif
