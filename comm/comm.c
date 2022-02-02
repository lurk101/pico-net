
#include "comm.h"

#include <stdlib.h>
#include <string.h>

#include "comm.pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"

#define PACKED __attribute__((__packed__))

#define COMM_PIO_CLK_DIV 10

typedef struct PACKED comm_pkt {
    uint16_t data_length;  // length of packet data in bytes
    uint8_t from;          // source node
    uint8_t to;            // destination node
    uint8_t data[0];       // message data
} comm_pkt_t;

typedef struct PACKED comm_buf {
    struct comm_buf* next; // next q element
    uint32_t length;       // length of receive in 32 bit words
    comm_pkt_t pkt;        // packet data
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
static int recv_dma_ctl_chan; // RX control channel
static int recv_dma_dat_chan; // RX data channel
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

static int comm_words(int bytes) { return (bytes + 3) / 4; }

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
    xmit_buf->length = comm_words(xmit_buf->pkt.data_length + sizeof(comm_pkt_t));
    dma_channel_set_trans_count(xmit_dma_chan, xmit_buf->length + 1, false);
    dma_channel_set_read_addr(xmit_dma_chan, &xmit_buf->length, true);
}

static void comm_recv_start(void) {
    // allocate a max size receive buffer
    recv_buf = malloc(4 * comm_words(sizeof(comm_buf_t) + COMM_PKT_SIZE));
    // the length will be written directly to the data DMA descriptor
    dma_channel_set_write_addr(recv_dma_dat_chan, &recv_buf->pkt, false);
    dma_channel_start(recv_dma_ctl_chan);
}

static void comm_dma_irq0_handler(void) {
    // Handle receive first since we may be enqueuing a transmit
    if (dma_channel_get_irq0_status(recv_dma_dat_chan)) {
        dma_irqn_acknowledge_channel(DMA_IRQ_0, recv_dma_dat_chan);
        // forward the message
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

#define COMM_RX_GPIO 5
#define COMM_TX_GPIO 4
#define COMM_BAUD 115200
#define COMM_SM 0
#define COMM_RX_PIO pio0
#define COMM_TX_PIO pio1
#define COMM_ID1_GPIO 2
#define COMM_ID0_GPIO 3

static void comm_default_recv_config(dma_channel_config* c, bool write_incr) {
    channel_config_set_transfer_data_size(c, DMA_SIZE_32);
    channel_config_set_read_increment(c, false);
    channel_config_set_write_increment(c, write_incr);
    channel_config_set_dreq(c, pio_get_dreq(COMM_RX_PIO, COMM_SM, false));
}

void comm_init(void) {
    gpio_pull_up(COMM_ID0_GPIO);
    gpio_pull_up(COMM_ID1_GPIO);
    gpio_set_dir(COMM_ID0_GPIO, false);
    gpio_set_dir(COMM_ID1_GPIO, false);
    busy_wait_us_32(100);
    host = (gpio_get(COMM_ID0_GPIO) ? 1 : 0) | (gpio_get(COMM_ID1_GPIO) ? 2 : 0);
    // uart
    uint offset = pio_add_program(COMM_RX_PIO, &uart_rx_program);
    uart_rx_program_init(COMM_RX_PIO, COMM_SM, offset, COMM_RX_GPIO, COMM_PIO_CLK_DIV);
    offset = pio_add_program(COMM_TX_PIO, &uart_tx_program);
    uart_tx_program_init(COMM_TX_PIO, COMM_SM, offset, COMM_TX_GPIO, COMM_PIO_CLK_DIV);
    // xmit
    xmit_q.head = xmit_q.tail = NULL; // initialize the tx q
    xmit_buf = NULL;
    xmit_dma_bsy = false;
    xmit_dma_chan = dma_claim_unused_channel(true); // get dma channel for tx
    dma_channel_config c = dma_channel_get_default_config(xmit_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32); // 32 bits per transfer
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c,
                            pio_get_dreq(COMM_TX_PIO, COMM_SM, true)); // UART TX dreq driven DMA
    dma_channel_set_irq0_enabled(xmit_dma_chan, true);       // interrupt when done
    // write to UART TX data register, no increment
    dma_channel_configure(xmit_dma_chan, &c, &COMM_TX_PIO->txf[COMM_SM], NULL, 1, false);

    // recv
    recv_q.head = recv_q.tail = NULL;
    recv_dma_ctl_chan = dma_claim_unused_channel(true);
    recv_dma_dat_chan = dma_claim_unused_channel(true);

    c = dma_channel_get_default_config(recv_dma_ctl_chan);
    comm_default_recv_config(&c, false);
    dma_channel_configure(recv_dma_ctl_chan, &c,
                          &dma_hw->ch[recv_dma_dat_chan].al1_transfer_count_trig,
                          &COMM_RX_PIO->rxf[COMM_SM], 1, false);

    c = dma_channel_get_default_config(recv_dma_dat_chan);
    comm_default_recv_config(&c, true);
    dma_channel_configure(recv_dma_dat_chan, &c, 0, &COMM_RX_PIO->rxf[COMM_SM], 0, false);
    dma_channel_set_irq0_enabled(recv_dma_dat_chan, true);

    irq_set_exclusive_handler(DMA_IRQ_0, comm_dma_irq0_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    comm_recv_start();
}

int comm_xmit(uint8_t to, const void* buffer, uint16_t length) {
    if (length < 1)
        return -1;
    if (length > COMM_PKT_SIZE)
        length = COMM_PKT_SIZE;
    comm_buf_t* buf = malloc(((sizeof(comm_buf_t) + length + 3) / 4) * 4);
    if (buf == NULL)
        return -1;
    buf->pkt.data_length = length;
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
    uint8_t l = buf->pkt.data_length;
    memcpy(buffer, buf->pkt.data, l);
    free(buf);
    return l;
}

uint8_t comm_host(void) { return host; }

void comm_deinit(void) {
    dma_channel_abort(recv_dma_ctl_chan);
    dma_channel_abort(recv_dma_dat_chan);
    dma_channel_abort(xmit_dma_chan);
    dma_channel_unclaim(recv_dma_ctl_chan);
    dma_channel_unclaim(recv_dma_dat_chan);
    dma_channel_unclaim(xmit_dma_chan);
}
