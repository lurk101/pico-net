#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define COMM_PKT_SIZE 256

void comm_init(void);
int comm_xmit(int to, const void* buffer, int packet_length);
int comm_recv_blocking(int* from, void* buffer, int buf_size);
int comm_recv_ready(void);
int comm_id(void);
int comm_baud(void);

#ifdef __cplusplus
}
#endif
