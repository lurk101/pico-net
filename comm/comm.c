
#include "comm.h"

#include <stdlib.h>
#include <string.h>

#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

typedef struct __attribute__((__packed__)) comm_hdr {
    struct comm_hdr* next;
    uint8_t length;
    uint8_t from;
    uint8_t to;
    uint8_t data[0];
} comm_hdr_t;

typedef struct comm_q {
    comm_hdr_t* head;
    comm_hdr_t* tail;
} comm_q_t;

static uint8_t host;

static comm_q_t xmit_q;
static int xmit_dma_chan;
static comm_hdr_t* xmit_buf;
static volatile bool xmit_dma_bsy;

static comm_q_t recv_q;
static int recv_dma_ctl_chan;
static int recv_dma_dat_chan;
static comm_hdr_t* recv_buf;

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
    if (xmit_q.head == NULL)
        xmit_q.tail = NULL;
    dma_channel_set_read_addr(xmit_dma_chan, &xmit_buf->length, false);
    dma_channel_set_trans_count(xmit_dma_chan, xmit_buf->length + 1, true);
}

static void recv_start_dma(void) {
    recv_buf = malloc(comm_max_packet_length + sizeof(comm_hdr_t));
    dma_channel_set_write_addr(recv_dma_dat_chan, &recv_buf->from, false);
    dma_channel_start(recv_dma_ctl_chan);
}

static void xmit_dma_irq0_handler(void) {
    if (dma_channel_get_irq0_status(recv_dma_dat_chan)) {
        dma_irqn_acknowledge_channel(DMA_IRQ_0, recv_dma_dat_chan);
        recv_buf->length = dma_debug_hw->ch[recv_dma_dat_chan].tcr;
        if (recv_buf->to == host) {
            if (recv_q.head == NULL)
                recv_q.head = recv_q.tail = recv_buf;
            else {
                recv_q.tail->next = recv_buf;
                recv_q.tail = recv_buf;
            }
        } else {
            if (xmit_q.head == NULL)
                xmit_q.head = xmit_q.tail = recv_buf;
            else {
                xmit_q.tail->next = recv_buf;
                xmit_q.tail = recv_buf;
            }
        }
        recv_start_dma();
    }
    if (dma_channel_get_irq0_status(xmit_dma_chan)) {
        dma_irqn_acknowledge_channel(DMA_IRQ_0, xmit_dma_chan);
        xmit_dma_bsy = false;
        xmit_start_dma();
    }
}

void comm_init(uint8_t host_addr) {
    host = host_addr;
    // uart
    gpio_set_function(4, GPIO_FUNC_UART); // set pin functions
    gpio_set_function(5, GPIO_FUNC_UART);
    uart_init(uart1, 115200);           // 115200 baud
    uart_set_fifo_enabled(uart1, true); // uart fifos enabled
    while (uart_is_readable(uart1))     // flush uart rx fifo
        uart_getc(uart1);
    // xmit
    xmit_q.head = xmit_q.tail = NULL; // initialize the tx q
    xmit_buf = NULL;
    xmit_dma_bsy = false;
    xmit_dma_chan = dma_claim_unused_channel(true); // get dma channel for tx
    dma_channel_config c = dma_channel_get_default_config(xmit_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8); // 8 bits per transfer
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, uart_get_dreq(uart1, true)); // UART TX dreq driven DMA
    dma_channel_set_irq0_enabled(xmit_dma_chan, true);       // interrupt when done
    // write to UART TX data register, no increment
    dma_channel_configure(xmit_dma_chan, &c, &uart_get_hw(uart1)->dr, NULL, 0, false);

    // recv
    recv_q.head = recv_q.tail = NULL;
    recv_dma_ctl_chan = dma_claim_unused_channel(true);
    recv_dma_dat_chan = dma_claim_unused_channel(true);

    c = dma_channel_get_default_config(recv_dma_ctl_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, uart_get_dreq(uart1, false));
    dma_channel_configure(recv_dma_ctl_chan, &c,
                          &dma_hw->ch[recv_dma_dat_chan].al1_transfer_count_trig,
                          &uart_get_hw(uart1)->dr, 1, false);

    c = dma_channel_get_default_config(recv_dma_dat_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, uart_get_dreq(uart1, false));
    dma_channel_configure(recv_dma_dat_chan, &c, 0, &uart_get_hw(uart1)->dr, 0, false);
    dma_channel_set_irq0_enabled(recv_dma_dat_chan, true);

    irq_set_exclusive_handler(DMA_IRQ_0, xmit_dma_irq0_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    recv_start_dma();
}

int comm_xmit(uint8_t to, const void* buffer, uint16_t length) {
    if ((length < 1) || (length > comm_max_packet_length))
        return -1;
    comm_hdr_t* hdr = malloc(sizeof(comm_hdr_t) + length);
    if (hdr == NULL)
        return -1;
    hdr->next = NULL;
    hdr->length = length + 2;
    hdr->from = host;
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
