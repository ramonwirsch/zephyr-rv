//
// Created by ray on 2/28/24.
//

#ifndef CGRA_OS_UART_ZIP_WBUART32_C_H
#define CGRA_OS_UART_ZIP_WBUART32_C_H

#define UART_ZIP_WBUART_IRQ_TX_SPACE 0x4
#define UART_ZIP_WBUART_IRQ_TX_HALF  0x8
#define UART_ZIP_WBUART_IRQ_RX_ANY   0x1
#define UART_ZIP_WBUART_IRQ_RX_HALF  0x2

struct uart_zip_wbuart32_data {
    uint32_t fifoStatCache;
    uint32_t rxErrorFlags;
    uint16_t maxTxBuf;
    uint16_t maxRxBuf;
    uint8_t irqMask;
};

void uart_zip_wbuart32_interrupt_enable(uint32_t baseAddr, struct uart_zip_wbuart32_data* data, uint8_t enable);

void uart_zip_wbuart32_interrupt_enable_masked(uint32_t baseAddr, struct uart_zip_wbuart32_data* data, uint8_t enable, uint8_t mask);

uint8_t uart_zip_wbuart32_interrupts_pending(uint32_t baseAddr);

int uart_zip_wbuart32_fifo_read_impl(uint32_t baseAddr, uint8_t *rx_data, int size);

int uart_zip_wbuart32_fifo_read(const struct device* dev, uint8_t *rx_data, int size);

int uart_zip_wbuart32_poll_in(const struct device *dev, unsigned char *c);
void uart_zip_wbuart32_poll_out(const struct device *dev, unsigned char c);
int uart_zip_wbuart32_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len);
int uart_zip_wbuart32_tx_complete(const struct device *dev);

uint8_t uart_zip_wbuart32_rx_avail(uint32_t baseAddr);

#endif //CGRA_OS_UART_ZIP_WBUART32_C_H
