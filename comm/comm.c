
#include <memory.h>
#include <stdlib.h>

#include "comm.h"
#include "comm.pio.h"

#include "hardware/dma.h"
#include "hardware/irq.h"
#include "pico/multicore.h"
#include "pico/sync.h"

#define PACKED __attribute__((__packed__))

// Message header
typedef struct PACKED packet {
    uint16_t data_length;  // length of packet data in bytes
    uint8_t from;          // source node
    uint8_t to;            // destination node
    uint8_t data[0];       // message data
} packet_t;

// Buffer
typedef struct PACKED buffer {
    struct buffer* next;   // next q element
    uint32_t length;       // length of receive in 32 bit words
    packet_t pkt;          // packet data
} buffer_t;

typedef volatile struct queue {
    buffer_t* head; // first node
    buffer_t* tail; // last node
} queue_t;

static uint8_t host_id; // This host's node id

static critical_section_t critical;
static queue_t free_q;

static queue_t xmit_q;             // pending transmissions
static int xmit_dma_chan;          // TX DMA channel
static buffer_t* xmit_buf;         // current TX buffer
static volatile bool xmit_dma_bsy; // TX busy state

static queue_t recv_q;        // received messages
static int recv_dma_ctl_chan; // RX control channel
static int recv_dma_dat_chan; // RX data channel
static buffer_t* recv_buf;    // current receive buffer

static void enqueue(queue_t* q, buffer_t* buf) {
    buf->next = NULL;
    if (q->head == NULL) // enqueue for retransmit
        q->head = q->tail = buf;
    else {
        q->tail->next = buf;
        q->tail = buf;
    }
}

static buffer_t* dequeue(queue_t* q) {
    if (q->head == NULL)
        return NULL;
    buffer_t* buf = q->head;
    q->head = q->head->next;
    if (q->head == NULL)
        q->tail = NULL;
    return buf;
}

static buffer_t* get_buffer() {
    buffer_t* buf = dequeue(&free_q);
    if (buf == NULL)
        buf = malloc(sizeof(buffer_t) + COMM_PKT_SIZE);
    return buf;
}

static int words(int bytes) { return (bytes + 3) / 4; }

static void start_xmit(void) {
    if (xmit_dma_bsy) // Already busy?
        return;
    if (xmit_buf) { // Free transmitted buffer
        enqueue(&free_q, xmit_buf);
        xmit_buf = NULL;
    }
    xmit_buf = dequeue(&xmit_q); // dequeue pending message
    if (xmit_buf == NULL)
        return;
    xmit_dma_bsy = true;
    // configure and start the TX DMA channel
    xmit_buf->length = words(xmit_buf->pkt.data_length + sizeof(packet_t));
    dma_channel_set_trans_count(xmit_dma_chan, xmit_buf->length + 1, false);
    dma_channel_set_read_addr(xmit_dma_chan, &xmit_buf->length, true);
}

static void start_recv(void) {
    // allocate a max size receive buffer
    recv_buf = get_buffer();
    if (recv_buf == NULL)
        abort();
    // the length will be written directly to the data DMA descriptor
    dma_channel_set_write_addr(recv_dma_dat_chan, &recv_buf->pkt, false);
    dma_channel_start(recv_dma_ctl_chan);
}

static void dma_irq0_handler(void) {
    // Handle receive first since we may be enqueuing a transmit
    if (dma_channel_get_irq0_status(recv_dma_dat_chan)) {
        dma_irqn_acknowledge_channel(DMA_IRQ_0, recv_dma_dat_chan);
        // forward the message
        critical_section_enter_blocking(&critical);
        enqueue(recv_buf->pkt.to == host_id ? &recv_q : &xmit_q, recv_buf);
        start_recv(); // restart for next message
        critical_section_exit(&critical);
    }
    // handle transmit
    if (dma_channel_get_irq0_status(xmit_dma_chan)) {
        dma_irqn_acknowledge_channel(DMA_IRQ_0, xmit_dma_chan);
        xmit_dma_bsy = false;
        critical_section_enter_blocking(&critical);
        start_xmit(); // restart TX if more left
        critical_section_exit(&critical);
    }
}

#define ID1_GPIO 2
#define ID0_GPIO 3
#define TX_GPIO 4
#define RX_GPIO 5
#define SM 0
#define RX_PIO pio0
#define TX_PIO pio1
// 125 MHz / 10 = 12.5 MHz
#define PIO_CLK_DIV 10

static void default_recv_dma_config(dma_channel_config* c, bool write_incr) {
    channel_config_set_transfer_data_size(c, DMA_SIZE_32);
    channel_config_set_read_increment(c, false);
    channel_config_set_write_increment(c, write_incr);
    channel_config_set_dreq(c, pio_get_dreq(RX_PIO, SM, false));
}

static void init_core1(void) {
    critical_section_init(&critical);
    gpio_pull_up(ID0_GPIO);
    gpio_pull_up(ID1_GPIO);
    gpio_set_dir(ID0_GPIO, false);
    gpio_set_dir(ID1_GPIO, false);
    busy_wait_us_32(10);
    host_id = (gpio_get(ID0_GPIO) ? 1 : 0) | (gpio_get(ID1_GPIO) ? 2 : 0);
    // uart
    uart_program_init(RX_PIO, TX_PIO, SM, RX_GPIO, TX_GPIO, PIO_CLK_DIV);
    // xmit
    free_q.head = free_q.tail = xmit_q.head = xmit_q.tail = recv_q.head = recv_q.tail = NULL;
    xmit_buf = NULL;
    xmit_dma_bsy = false;
    xmit_dma_chan = dma_claim_unused_channel(true); // get dma channel for tx
    dma_channel_config c = dma_channel_get_default_config(xmit_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32); // 32 bits per transfer
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(TX_PIO, SM, true)); // UART TX dreq driven DMA
    dma_channel_set_irq0_enabled(xmit_dma_chan, true);       // interrupt when done
    // write to UART TX data register, no increment
    dma_channel_configure(xmit_dma_chan, &c, &TX_PIO->txf[SM], NULL, 1, false);

    // recv
    recv_dma_ctl_chan = dma_claim_unused_channel(true);
    recv_dma_dat_chan = dma_claim_unused_channel(true);

    c = dma_channel_get_default_config(recv_dma_ctl_chan);
    default_recv_dma_config(&c, false);
    dma_channel_configure(recv_dma_ctl_chan, &c,
                          &dma_hw->ch[recv_dma_dat_chan].al1_transfer_count_trig, &RX_PIO->rxf[SM],
                          1, false);

    c = dma_channel_get_default_config(recv_dma_dat_chan);
    default_recv_dma_config(&c, true);
    dma_channel_configure(recv_dma_dat_chan, &c, 0, &RX_PIO->rxf[SM], 0, false);
    dma_channel_set_irq0_enabled(recv_dma_dat_chan, true);

    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq0_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    critical_section_enter_blocking(&critical);
    start_recv();
    critical_section_exit(&critical);
    multicore_fifo_push_blocking(0);
    for (;;)
        __wfi();
}

void comm_init(void) {
    multicore_launch_core1(init_core1);
    multicore_fifo_pop_blocking();
}

int comm_xmit(uint8_t to, const void* buffer, uint16_t length) {
    if (length < 1)
        return -1;
    if (length > COMM_PKT_SIZE)
        length = COMM_PKT_SIZE;
    critical_section_enter_blocking(&critical);
    buffer_t* buf = get_buffer();
    if (buf == NULL) {
        critical_section_exit(&critical);
        return -1;
    }
    buf->pkt.data_length = length;
    buf->pkt.from = host_id;
    buf->pkt.to = to;
    memcpy(buf->pkt.data, buffer, length);
#if COMM_FORCE_PIO
    enqueue(&xmit_q, buf);
#else
    enqueue(buf->pkt.to == host_id ? &recv_q : &xmit_q, buf);
#endif
    start_xmit();
    critical_section_exit(&critical);
    return length;
}

int comm_recv(uint8_t* from, void* buffer) {
    if (recv_q.head == NULL)
        return -1;
    critical_section_enter_blocking(&critical);
    buffer_t* buf = dequeue(&recv_q);
    *from = buf->pkt.from;
    uint8_t l = buf->pkt.data_length;
    memcpy(buffer, buf->pkt.data, l);
    enqueue(&free_q, buf);
    critical_section_exit(&critical);
    return l;
}

uint8_t comm_host_id(void) { return host_id; }
