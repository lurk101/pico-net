
#include <limits.h>
#include <memory.h>
#include <stdlib.h>

#include "comm.h"
#include "comm.pio.h"

#include "hardware/dma.h"
#include "hardware/irq.h"

#include "pico/multicore.h"
#include "pico/platform.h"
#include "pico/sync.h"

#define PACKED __attribute__((__packed__))

#define ID1_GPIO 2     // bit 1 of node id
#define ID0_GPIO 3     // bit 0 of node id
#define TX_GPIO 4      // TX pin
#define RX_GPIO 5      // RX pin
#define LED_GPIO 25    // LED pin
#define SM 0           // PIO state machine 0
#define RX_PIO pio0    // RX PIO
#define TX_PIO pio1    // RX PIO
#define PIO_CLK_DIV 10 // 125 MHz / 10 = 12.5 MHz (8 clks / bit)

// Message header
typedef struct PACKED packet {
    uint16_t data_length;  // length of packet data in bytes
    uint8_t from;          // source node
    uint8_t to;            // destination node
    uint8_t data[0];       // message data
} packet_t;

// Buffer
typedef struct PACKED buffer {
    struct buffer* next;   // next q entry
    uint32_t length;       // length of receive in 32 bit words
    packet_t pkt;          // packet data
} buffer_t;

// Queue root
typedef volatile struct queue {
    buffer_t* head; // first entry
    buffer_t* tail; // last entry
} queue_t;

static uint8_t id; // This node's id
static spin_lock_t* lock;

static queue_t free_q; // free buffer pool

static queue_t xmit_q;         // pending transmissions
static int xmit_dma_chan;      // TX DMA channel
static buffer_t* xmit_buf;     // current TX buffer

static queue_t recv_q;        // received messages
static int recv_dma_ctl_chan; // RX control channel
static int recv_dma_dat_chan; // RX data channel
static buffer_t* recv_buf;    // current receive buffer
static semaphore_t recv_sem;  // receive synchronization

// push back
static void enqueue(queue_t* q, buffer_t* buf) {
    buf->next = NULL;
    if (q->head == NULL) // enqueue for retransmit
        q->head = q->tail = buf;
    else {
        q->tail->next = buf;
        q->tail = buf;
    }
}

// pop front
static buffer_t* dequeue(queue_t* q) {
    if (q->head == NULL)
        return NULL;
    buffer_t* buf = q->head;
    q->head = q->head->next;
    if (q->head == NULL)
        q->tail = NULL;
    return buf;
}

// Get buffer from free q. If empty the allocate a new one
static buffer_t* get_buffer() {
    buffer_t* buf = dequeue(&free_q);
    if (buf == NULL)
        buf = malloc(sizeof(buffer_t) + COMM_PKT_SIZE);
    return buf;
}

// Round bytes up to full number of 32 bit words
static int words(int bytes) { return (bytes + 3) / 4; }

// Start transmit of 1st entry in tx q
static void start_xmit(void) {
    if (gpio_get(LED_GPIO)) // Already busy?
        return;   // DMA interrupt will restart transmit
    if (xmit_buf) // Free transmitted buffer
        enqueue(&free_q, xmit_buf);
    xmit_buf = dequeue(&xmit_q); // dequeue pending message
    if (xmit_buf == NULL)
        return;
    gpio_put(LED_GPIO, 1);
    // configure and start the TX DMA channel
    xmit_buf->length = words(xmit_buf->pkt.data_length + sizeof(packet_t));
    dma_channel_set_trans_count(xmit_dma_chan, xmit_buf->length + 1, false);
    dma_channel_set_read_addr(xmit_dma_chan, &xmit_buf->length, true);
}

// Start the 1 tx dma to retrieve the buffer length into the 2 tx dma descriptor
static void start_recv(void) {
    // allocate a max size receive buffer
    recv_buf = get_buffer();
    if (recv_buf == NULL)
        panic("Out of comm buffers");
    // the length will be written directly to the data DMA descriptor
    dma_channel_set_write_addr(recv_dma_dat_chan, &recv_buf->pkt, false);
    dma_channel_start(recv_dma_ctl_chan);
}

// Handle TX or RX dma done interrupt
static void dma_irq0_handler(void) {
    // Handle receive first since we may be enqueuing a transmit
    if (dma_channel_get_irq0_status(recv_dma_dat_chan)) {
        dma_irqn_acknowledge_channel(DMA_IRQ_0, recv_dma_dat_chan);
        // forward the message
        uint status = spin_lock_blocking(lock);
        if (recv_buf->pkt.to == id) {
            enqueue(&recv_q, recv_buf);
            sem_release(&recv_sem);
        } else
            enqueue(&xmit_q, recv_buf);
        start_recv(); // restart for next message
        spin_unlock(lock, status);
    }
    // handle transmit
    if (dma_channel_get_irq0_status(xmit_dma_chan)) {
        dma_irqn_acknowledge_channel(DMA_IRQ_0, xmit_dma_chan);
        gpio_put(LED_GPIO, 0);
        uint status = spin_lock_blocking(lock);
        start_xmit(); // restart TX if more left
        spin_unlock(lock, status);
    }
}

static void default_recv_dma_config(dma_channel_config* c, bool write_incr) {
    channel_config_set_transfer_data_size(c, DMA_SIZE_32);
    channel_config_set_read_increment(c, false);
    channel_config_set_write_increment(c, write_incr);
    channel_config_set_dreq(c, pio_get_dreq(RX_PIO, SM, false));
}

static void init_core1(void) {
    gpio_pull_up(ID0_GPIO);
    gpio_pull_up(ID1_GPIO);
    gpio_set_dir(ID0_GPIO, GPIO_IN);
    gpio_set_dir(ID1_GPIO, GPIO_IN);
    gpio_init(LED_GPIO);
    gpio_set_dir(LED_GPIO, GPIO_OUT);
    gpio_put(LED_GPIO, 0);
    busy_wait_us_32(10);
    id = (gpio_get(ID0_GPIO) ? 1 : 0) | (gpio_get(ID1_GPIO) ? 2 : 0);
    // uart
    uart_program_init(RX_PIO, TX_PIO, SM, RX_GPIO, TX_GPIO, PIO_CLK_DIV);
    // xmit
    free_q.head = free_q.tail = xmit_q.head = xmit_q.tail = recv_q.head = recv_q.tail = NULL;
    sem_init(&recv_sem, 0, SHRT_MAX);
    xmit_buf = NULL;
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

    lock = spin_lock_init(spin_lock_claim_unused(true));
    uint status = spin_lock_blocking(lock);
    start_recv();
    spin_unlock(lock, status);
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
    uint status = spin_lock_blocking(lock);
    buffer_t* buf = get_buffer();
    if (buf == NULL) {
        spin_unlock(lock, status);
        return -1;
    }
    buf->pkt.data_length = length;
    buf->pkt.from = id;
    buf->pkt.to = to;
    memcpy(buf->pkt.data, buffer, length);
#if COMM_FORCE_PIO
    enqueue(&xmit_q, buf);
#else
    if (buf->pkt.to == id) {
        enqueue(&recv_q, buf);
        sem_release(&recv_sem);
    } else
        enqueue(&xmit_q, buf);
#endif
    start_xmit();
    spin_unlock(lock, status);
    return length;
}

int comm_recv_ready(void) { return recv_q.head != NULL; }

int comm_recv_blocking(uint8_t* from, void* buffer) {
    sem_acquire_blocking(&recv_sem);
    uint status = spin_lock_blocking(lock);
    buffer_t* buf = dequeue(&recv_q);
    *from = buf->pkt.from;
    uint8_t l = buf->pkt.data_length;
    memcpy(buffer, buf->pkt.data, l);
    enqueue(&free_q, buf);
    spin_unlock(lock, status);
    return l;
}

uint8_t comm_id(void) { return id; }
