
#include "comm.h"

#include <stdlib.h>
#include <string.h>

#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

typedef struct __attribute__((__packed__)) comm_pkt {
    uint8_t length;        // data length (including source and destination)
    uint8_t from;          // source node
    uint8_t to;            // destination node
    uint8_t data[COMM_PKT_SIZE]; // message data
} comm_pkt_t;

typedef struct __attribute__((__packed__)) comm_buf {
    struct comm_buf* next; // next q element
    comm_pkt_t pkt;
} comm_buf_t;

typedef volatile struct comm_q {
    comm_buf_t* head; // first node
    comm_buf_t* tail; // last node
} comm_q_t;

static uint8_t host; // This host's node id

static comm_q_t xmit_q;            // pending transmissions
static int xmit_dma_chan;          // TX DMA channel
static comm_buf_t* xmit_buf;       // current TX buffer
static volatile bool xmit_dma_bsy; // TX busy state

static comm_q_t recv_q;       // received messages
static int recv_dma_chan;     // RX data channel
static comm_buf_t* recv_buf;  // current receive buffer

static void comm_enqueue(comm_q_t* q, comm_buf_t* buf) {
    buf->next = NULL;
    if (q->head == NULL) // enqueue for retransmit
        q->head = q->tail = buf;
    else {
        q->tail->next = buf;
        q->tail = buf;
    }
}

static comm_buf_t* comm_dequeue(comm_q_t* q) {
    if (q->head == NULL)
        return NULL;
    comm_buf_t* buf = q->head;
    q->head = q->head->next;
    if (q->head == NULL)
        q->tail = NULL;
    return buf;
}

static void comm_xmit_start(void) {
    if (xmit_dma_bsy) // Already busy?
        return;
    if (xmit_buf) { // Free transmitted buffer
        free(xmit_buf);
        xmit_buf = NULL;
    }
    xmit_buf = comm_dequeue(&xmit_q); // dequeue pending message
    if (xmit_buf == NULL)
        return;
    xmit_dma_bsy = true;
    // configure and start the TX DMA channel
    dma_channel_set_read_addr(xmit_dma_chan, xmit_buf, true);
}

static void comm_recv_start(void) {
    // allocate a max size receive buffer
    recv_buf = malloc(sizeof(comm_buf_t));
    // the length will be written directly to the data DMA descriptor
    dma_channel_set_write_addr(recv_dma_chan, recv_buf, true);
}

static void comm_dma_irq0_handler(void) {
    // Handle receive first since we may be enqueuing a transmit
    if (dma_channel_get_irq0_status(recv_dma_chan)) {
        dma_irqn_acknowledge_channel(DMA_IRQ_0, recv_dma_chan);
        // retrieve the message length
        comm_enqueue(recv_buf->pkt.to == host ? &recv_q : &xmit_q, recv_buf);
        comm_recv_start(); // restart for next message
    }
    // handle transmit
    if (dma_channel_get_irq0_status(xmit_dma_chan)) {
        dma_irqn_acknowledge_channel(DMA_IRQ_0, xmit_dma_chan);
        xmit_dma_bsy = false;
        comm_xmit_start(); // restart TX if more left
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
    dma_channel_configure(xmit_dma_chan, &c, &uart_get_hw(uart1)->dr, NULL, sizeof(comm_pkt_t),
                          false);

    // recv
    recv_q.head = recv_q.tail = NULL;
    recv_dma_chan = dma_claim_unused_channel(true);

    c = dma_channel_get_default_config(recv_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, uart_get_dreq(uart1, false));
    dma_channel_configure(recv_dma_chan, &c, 0, &uart_get_hw(uart1)->dr, sizeof(comm_pkt_t), false);
    dma_channel_set_irq0_enabled(recv_dma_chan, true);

    irq_set_exclusive_handler(DMA_IRQ_0, comm_dma_irq0_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    comm_recv_start();
}

int comm_xmit(uint8_t to, const void* buffer, uint16_t length) {
    if (length < 1)
        return -1;
    if (length > COMM_PKT_SIZE)
        length = COMM_PKT_SIZE;
    comm_buf_t* buf = malloc(sizeof(comm_buf_t));
    if (buf == NULL)
        return -1;
    buf->pkt.length = length - 1;
    buf->pkt.from = host;
    buf->pkt.to = to;
    memcpy(buf->pkt.data, buffer, length);
    irq_set_enabled(DMA_IRQ_0, false);
    comm_enqueue(buf->pkt.to == host ? &recv_q : &xmit_q, buf);
    // comm_enqueue(&xmit_q, buf);
    comm_xmit_start();
    irq_set_enabled(DMA_IRQ_0, true);
    return length;
}

int comm_recv(uint8_t* from, void* buffer) {
    if (recv_q.head == NULL)
        return -1;
    irq_set_enabled(DMA_IRQ_0, false);
    comm_buf_t* buf = comm_dequeue(&recv_q);
    irq_set_enabled(DMA_IRQ_0, true);
    *from = buf->pkt.from;
    uint8_t l = buf->pkt.length + 1;
    memcpy(buffer, buf->pkt.data, l);
    free(buf);
    return l;
}
