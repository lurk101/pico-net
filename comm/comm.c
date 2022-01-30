
#include "comm.h"

#include <stdlib.h>
#include <string.h>

#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

typedef struct comm_hdr {
    struct comm_hdr* next;
    uint16_t length;
    uint8_t from;
    uint8_t to;
    uint8_t data[0];
} comm_hdr_t;

typedef struct comm_q {
    comm_hdr_t* head;
    comm_hdr_t* tail;
} comm_q_t;

static comm_q_t xmit_q;
static int xmit_dma_chan;
static comm_hdr_t* xmit_buf;
static volatile bool xmit_dma_bsy;

static void xmit_start_dma(void) {
    if (xmit_dma_bsy)
        return;
    if (xmit_buf) {
        free(xmit_buf);
        xmit_buf = NULL;
    }
    if (xmit_q.head == NULL)
        return;
    xmit_dma_bsy = true;
    xmit_buf = xmit_q.head;
    xmit_q.head = xmit_q.head->next;
    dma_channel_set_read_addr(xmit_dma_chan, &xmit_buf->length, false);
    dma_channel_set_trans_count(xmit_dma_chan, xmit_buf->length + 2, true);
}

static void xmit_dma_irq0_handler(void) {
    dma_channel_acknowledge_irq0(xmit_dma_chan);
    xmit_dma_bsy = false;
    xmit_start_dma();
}

void comm_init(void) {
	uart_init(uart1, 115200);
    xmit_q.head = xmit_q.tail = NULL;
    xmit_buf = NULL;
    xmit_dma_bsy = false;
    xmit_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(xmit_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, uart_get_dreq(uart1, true));
    dma_channel_set_irq0_enabled(xmit_dma_chan, true);
    dma_channel_configure(xmit_dma_chan, &c, &uart_get_hw(uart1)->dr, NULL, 0, false);
    irq_set_exclusive_handler(DMA_IRQ_0, xmit_dma_irq0_handler);
	irq_set_enabled(DMA_IRQ_0, true);
}

int comm_xmit(uint8_t from, uint8_t to, const void* buffer, uint16_t length) {
	if (length > comm_max_packet_length)
		return -1;
    comm_hdr_t* hdr = malloc(sizeof(comm_hdr_t) + length);
    if (hdr == NULL)
        return -1;
    hdr->next = NULL;
    hdr->length = length + 2;
    hdr->from = from;
    hdr->to = to;
    memcpy(hdr->data, buffer, length);
	irq_set_enabled(DMA_IRQ_0, false);
    if (xmit_q.head == NULL)
        xmit_q.head = xmit_q.tail = hdr;
    else {
        xmit_q.tail->next = hdr;
        xmit_q.tail = hdr;
    }
    xmit_start_dma();
	irq_set_enabled(DMA_IRQ_0, true);
    return length;
}

int comm_recv(uint8_t* from, void* buffer, uint16_t length) {}
