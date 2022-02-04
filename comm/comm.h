#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define COMM_PKT_SIZE 256 // Maximum packet size

void comm_init(void);
int comm_xmit(int to, const void* buffer, int packet_length);  // send packet
int comm_recv_blocking(int* from, void* buffer, int buf_size); // receive packet
int comm_recv_ready(void);                                     // Packet available ?
int comm_id(void);                                             // Get node id
int comm_baud(void);                                           // Get effective baud rate

#ifdef __cplusplus
}
#endif
