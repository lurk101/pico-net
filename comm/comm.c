/* Copyright (C) 1883 Thomas Edison - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GPLv2 license, which unfortunately won't be
 * written for another century.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <limits.h>
#include <memory.h>
#include <stdlib.h>

#include "comm.h"
#include "comm.pio.h"

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

#include "pico/multicore.h"
#include "pico/platform.h"
#include "pico/sync.h"

#define PACKED __attribute__((__packed__))

#define ID1_GPIO 2     // bit 1 of 2 bit node id
#define ID0_GPIO 3     // bit 0 of node id
#define TX_GPIO 4      // pio UART TX pin
#define RX_GPIO 5      // pio UART RX pin
#define LED_GPIO 25    // LED pin, also xmit busy flag
#define SM 0           // using PIO state machine 0
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

static queue_t xmit_q;     // pending transmissions
static int xmit_dma_chan;  // TX DMA channel
static buffer_t* xmit_buf; // current TX buffer

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

// Get buffer from free q. If empty then allocate a new one
static buffer_t* get_buffer() {
    buffer_t* buf = dequeue(&free_q);
    if (buf == NULL)
        buf = malloc(sizeof(buffer_t) + COMM_PKT_SIZE);
    return buf;
}

// Start transmit of 1st entry in tx q
static void start_xmit(void) {
    if (gpio_get(LED_GPIO)) // Already busy?
        return;   // DMA interrupt will restart transmit
    if (xmit_buf) // Free transmitted buffer
        enqueue(&free_q, xmit_buf);
    xmit_buf = dequeue(&xmit_q); // dequeue pending message
    if (xmit_buf == NULL)
        return;
    gpio_put(LED_GPIO, 1); // xmit busy
    // configure and start the TX DMA channel
    // temporarilly use next pointer as word count
    uint words = ((xmit_buf->pkt.data_length + sizeof(packet_t) + 3) / 4) * 4;
    xmit_buf->next = (buffer_t*)words;
    dma_channel_set_trans_count(xmit_dma_chan, words + 1, false);
    dma_channel_set_read_addr(xmit_dma_chan, xmit_buf, true);
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

static void common_recv_dma_config(dma_channel_config* c) {
    channel_config_set_transfer_data_size(c, DMA_SIZE_32);
    channel_config_set_read_increment(c, false);
    channel_config_set_dreq(c, pio_get_dreq(RX_PIO, SM, false));
}

static void init_core1(void) {
    // read node id
    gpio_pull_up(ID0_GPIO);
    gpio_pull_up(ID1_GPIO);
    gpio_set_dir(ID0_GPIO, GPIO_IN);
    gpio_set_dir(ID1_GPIO, GPIO_IN);
    gpio_init(LED_GPIO);
    gpio_set_dir(LED_GPIO, GPIO_OUT);
    gpio_put(LED_GPIO, 0);
    busy_wait_us_32(10);
    id = (gpio_get(ID0_GPIO) ? 1 : 0) | (gpio_get(ID1_GPIO) ? 2 : 0);

    // 32 bit PIO UART
    // Tell PIO to initially drive output-high on the selected pin, then map PIO
    // onto that pin with the IO muxes.
    pio_sm_set_pins_with_mask(TX_PIO, SM, 1u << TX_GPIO, 1u << TX_GPIO);
    pio_sm_set_pindirs_with_mask(TX_PIO, SM, 1u << TX_GPIO, 1u << TX_GPIO);
    pio_gpio_init(TX_PIO, TX_GPIO);
    uint offset = pio_add_program(TX_PIO, &uart_tx_program);
    pio_sm_config p = uart_tx_program_get_default_config(offset);
    // OUT shifts to right, no autopull
    sm_config_set_out_shift(&p, true, false, 32);
    // We are mapping both OUT and side-set to the same pin, because sometimes
    // we need to assert user data onto the pin (with OUT) and sometimes
    // assert constant values (start/stop bit)
    sm_config_set_out_pins(&p, TX_GPIO, 1);
    sm_config_set_sideset_pins(&p, TX_GPIO);
    sm_config_set_clkdiv(&p, PIO_CLK_DIV);
    pio_sm_init(TX_PIO, SM, offset, &p);
    pio_sm_set_enabled(TX_PIO, SM, true);
    busy_wait_us_32(250000);
    pio_sm_set_consecutive_pindirs(RX_PIO, SM, RX_GPIO, 1, false);
    pio_gpio_init(RX_PIO, RX_GPIO);
    gpio_pull_up(RX_GPIO);
    offset = pio_add_program(RX_PIO, &uart_rx_program);
    p = uart_rx_program_get_default_config(offset);
    sm_config_set_in_pins(&p, RX_GPIO); // for WAIT, IN
    // Shift to right, autopush enabled
    sm_config_set_in_shift(&p, true, true, 32);
    // SM transmits 1 bit per 8 execution cycles.
    sm_config_set_clkdiv(&p, PIO_CLK_DIV);
    pio_sm_init(RX_PIO, SM, offset, &p);
    pio_sm_set_enabled(RX_PIO, SM, true);

    // xmit DMA
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

    // recv DMA
    recv_dma_ctl_chan = dma_claim_unused_channel(true);
    recv_dma_dat_chan = dma_claim_unused_channel(true);
    // control channel
    c = dma_channel_get_default_config(recv_dma_ctl_chan);
    common_recv_dma_config(&c);
    channel_config_set_write_increment(&c, false);
    dma_channel_configure(recv_dma_ctl_chan, &c,
                          &dma_hw->ch[recv_dma_dat_chan].al1_transfer_count_trig, &RX_PIO->rxf[SM],
                          1, false);
    // data channel
    c = dma_channel_get_default_config(recv_dma_dat_chan);
    common_recv_dma_config(&c);
    channel_config_set_write_increment(&c, true);
    dma_channel_configure(recv_dma_dat_chan, &c, 0, &RX_PIO->rxf[SM], 0, false);
    dma_channel_set_irq0_enabled(recv_dma_dat_chan, true);

    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq0_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    lock = spin_lock_init(spin_lock_claim_unused(true));
    uint status = spin_lock_blocking(lock);
    start_recv();
    spin_unlock(lock, status);
    multicore_fifo_push_blocking(0); // tell core 0 we're ready
    for (;;)                         // handle interrupts
        __wfi();
}

// Launch core 1 to initialize communication
void comm_init(void) {
    multicore_launch_core1(init_core1);
    multicore_fifo_pop_blocking(); // wait for core 1 initialized
}

// transmit a packet
int comm_xmit(int to, const void* buffer, int length) {
    if (length > COMM_PKT_SIZE)
        length = COMM_PKT_SIZE;
    uint status = spin_lock_blocking(lock);
    buffer_t* buf = get_buffer();
    spin_unlock(lock, status);
    if (buf == NULL)
        return -1;
    buf->pkt.data_length = length;
    buf->pkt.from = id;
    buf->pkt.to = to;
    if (buffer && length)
        memcpy(buf->pkt.data, buffer, length);
    status = spin_lock_blocking(lock);
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

// Receive ready ?
int comm_recv_ready(void) { return recv_q.head != NULL; }

// Receive a packet
int comm_recv_blocking(int* from, void* buffer, int buf_length) {
    sem_acquire_blocking(&recv_sem);
    uint status = spin_lock_blocking(lock);
    buffer_t* buf = dequeue(&recv_q);
    spin_unlock(lock, status);
    if (from)
        *from = buf->pkt.from;
    uint8_t l = buf->pkt.data_length;
    if (l > buf_length)
        l = buf_length;
    if (buffer && l)
        memcpy(buffer, buf->pkt.data, l);
    status = spin_lock_blocking(lock);
    enqueue(&free_q, buf);
    spin_unlock(lock, status);
    return l;
}

// return node's id
int comm_id(void) { return id; }

// return effective communication baud rate (8 PIO clocks per bit)
int comm_baud(void) { return clock_get_hz(clk_sys) / (PIO_CLK_DIV * 8); }
