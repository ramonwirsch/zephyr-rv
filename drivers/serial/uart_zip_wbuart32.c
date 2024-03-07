/*
 * Copyright (c) 2018 Foundries.io
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zip_wbuart32

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <soc.h>
#include <zephyr/drivers/uart/uart_zip_wbuart32.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zWbUart, LOG_LEVEL_ERR);

#define TX_BUFFER_HAS_SPACE 0x00010000
#define RX_BUFFER_HAS_DATA  0x00000001
#define LGLEN_MASK          0xF
#define TX_FILL_LEVEL_SHIFT 18
#define RX_FILL_LEVEL_SHIFT 2
#define TX_LGNL_SHIFT		28
#define RX_LGNL_SHIFT		12
#define FILL_LEVEL_MASK     0x3FF
#define RX_DAT_INVALID      0x00000100
#define RX_DAT_ERROR        0x00001600
#define RX_DAT_ERROR_FIFO_OVFL    0x00001000
#define RX_DAT_ERROR_FRAME  0x00000400
#define RX_DAT_ERROR_PARITY 0x00000200
#define TX_DAT_NOT_FULL (1 << 13)

#define UART_SETUP_REG 0x0
#define UART_STATUS_REG 0x4
#define UART_RX_REG 0x8
#define UART_TX_REG 0xC
#define UART_INTR_REG 0x10

struct uart_zip_wbuart32_config {
	uint32_t base;
	uint32_t sysFreq;
	uint32_t baudrate;
    uint8_t rxLglen;
    uint8_t txLglen;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif
};

int uart_zip_wbuart32_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_zip_wbuart32_config *config = dev->config;
	
	uint32_t rx = sys_read32(config->base + UART_RX_REG);


	if ((rx & RX_DAT_INVALID) == 0) {
		*c = rx & 0xFF;
		return 0;
	} else if ((rx & RX_DAT_ERROR) == 0) {
		return -1;
	} else {
		struct uart_zip_wbuart32_data* data = (struct uart_zip_wbuart32_data*)dev->data;
		data->rxErrorFlags = rx;
		return -1;
	}
}

void uart_zip_wbuart32_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_zip_wbuart32_config *config = dev->config;
	uint32_t baseAddr = config->base;

	while (!(sys_read32(baseAddr + UART_STATUS_REG) & TX_BUFFER_HAS_SPACE)) {}

	sys_write32(c, baseAddr + UART_TX_REG);
}



int uart_zip_wbuart32_err_check(const struct device *dev)
{
	struct uart_zip_wbuart32_data* data = (struct uart_zip_wbuart32_data*)dev->data;

	int err = 0;

	if (data->rxErrorFlags & RX_DAT_ERROR_FIFO_OVFL) {
		err |= UART_ERROR_OVERRUN;
	}

	if (data->rxErrorFlags & RX_DAT_ERROR_FRAME) {
		err |= UART_ERROR_FRAMING;
	}

	if (data->rxErrorFlags & RX_DAT_ERROR_PARITY) {
		err |= UART_ERROR_PARITY;
	}

	data->rxErrorFlags = 0;
	
	return err;
}

int uart_zip_wbuart32_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	const struct uart_zip_wbuart32_config *config = dev->config;
	uint32_t baseAddr = config->base;

	uint32_t fifoStat = sys_read32(baseAddr + UART_STATUS_REG);

	int freeSlots = (fifoStat >> TX_FILL_LEVEL_SHIFT) & FILL_LEVEL_MASK;
	int byteCount = MIN(freeSlots, len);

	for (int i=0; i < byteCount; ++i) {
		uint8_t c = *tx_data++;
		sys_write32(c, baseAddr + UART_TX_REG);
	}

	return byteCount;
}

int uart_zip_wbuart32_fifo_read_impl(uint32_t baseAddr, uint8_t *rx_data, int size) {

    uint32_t fifoStat = sys_read32(baseAddr + UART_STATUS_REG);

    int avail = (fifoStat >> RX_FILL_LEVEL_SHIFT) & FILL_LEVEL_MASK;
    int byteCount = MIN(size, avail);

    int i;
    for (i=0; i < byteCount; ++i) {
        uint32_t d = sys_read32(baseAddr + UART_RX_REG);
        if ((d & RX_DAT_INVALID) == 0) {
            *rx_data++ = d & 0xFF;
        } else {
            break;
        }
    }

    return i;
}

int uart_zip_wbuart32_fifo_read(const struct device *dev, uint8_t *rx_data, int size) {
	const struct uart_zip_wbuart32_config *config = dev->config;
	uint32_t baseAddr = config->base;

	return uart_zip_wbuart32_fifo_read_impl(baseAddr, rx_data, size);
}

int uart_zip_wbuart32_tx_complete(const struct device* dev) {
	const struct uart_zip_wbuart32_config* config = dev->config;
	const struct uart_zip_wbuart32_data* data = dev->data;

	uint32_t fifoStat = sys_read32(config->base + UART_STATUS_REG);

	int freeSpaces = (fifoStat >> TX_FILL_LEVEL_SHIFT) & FILL_LEVEL_MASK;
	return freeSpaces == data->maxTxBuf;
}

int uart_zip_wbuart32_tx_ready(const struct device* dev) {
	const struct uart_zip_wbuart32_config* config = dev->config;

	uint32_t fifoStat = sys_read32(config->base + UART_STATUS_REG);

	return (fifoStat & TX_BUFFER_HAS_SPACE) != 0;
}

uint8_t uart_zip_wbuart32_rx_avail(uint32_t baseAddr) {
    uint32_t fifoStat = sys_read32(baseAddr + UART_STATUS_REG);
    return ((fifoStat >> RX_FILL_LEVEL_SHIFT) & FILL_LEVEL_MASK);
}

int uart_zip_wbuart32_rx_ready(const struct device* dev) {
	const struct uart_zip_wbuart32_config* config = dev->config;

    return uart_zip_wbuart32_rx_avail(config->base) != 0;
}

void uart_zip_wbuart32_interrupt_enable(uint32_t baseAddr, struct uart_zip_wbuart32_data* data, uint8_t enable) {
    uint8_t newMask = data->irqMask |= enable;
    sys_write32(newMask, baseAddr + UART_INTR_REG);
}

void uart_zip_wbuart32_interrupt_enable_masked(uint32_t baseAddr, struct uart_zip_wbuart32_data* data, uint8_t enable, uint8_t mask) {
    uint8_t newMask = (data->irqMask & ~mask) | (enable & mask);
    data->irqMask = newMask;
    sys_write32(newMask, baseAddr+UART_INTR_REG);
}

uint8_t uart_zip_wbuart32_interrupts_pending(uint32_t baseAddr) {
    uint32_t intr = sys_read32(baseAddr + UART_INTR_REG);
    return (intr >> 8) & 0xFF;
}

static void uart_zip_wbuart32_isr(const struct device *dev) {

}

static int uart_zip_wbuart32_init(const struct device *dev)
{
	const struct uart_zip_wbuart32_config* config = dev->config;
	struct uart_zip_wbuart32_data* data = dev->data;

	uint32_t fifoStat = sys_read32(config->base + UART_STATUS_REG);
	int rxLgnl = (fifoStat >> RX_LGNL_SHIFT) & LGLEN_MASK;
	data->maxRxBuf = 1 << rxLgnl;
	int txLgnl = (fifoStat >> TX_LGNL_SHIFT) & LGLEN_MASK;
	data->maxTxBuf = (1 << txLgnl) -1;

    if (rxLgnl != config->rxLglen || txLgnl != config->txLglen) {
        LOG_ERR("DT does not match device tree for dev 0x%08x: DTvsAct RX 0x%x 0x%x TX 0x%x 0x%x", (uint32_t)dev, config->rxLglen, rxLgnl, config->txLglen, txLgnl);
        k_fatal_halt(0x77627561); // spells "wbua" in ascii
    }


#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif

	return 0;
}

static const struct uart_driver_api uart_zip_wbuart32_driver_api = {
	.poll_in = uart_zip_wbuart32_poll_in,
	.poll_out = uart_zip_wbuart32_poll_out,
	.err_check = uart_zip_wbuart32_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_tx_complete = uart_zip_wbuart32_tx_complete, // works without actually using interrupts, but structs must still be compiled for it
	.irq_tx_ready = uart_zip_wbuart32_tx_ready,
	.irq_rx_ready = uart_zip_wbuart32_rx_ready,
	.fifo_fill = uart_zip_wbuart32_fifo_fill,
	.fifo_read = uart_zip_wbuart32_fifo_read,
#endif
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_ZIP_WBUART32_IRQ_SETUP_DECL(index)				\
	static void uart_zip_wbuart32_irq_config_func_##index(const struct device *dev);

#define UART_ZIP_WBUART32_IRQ_SETUP(index)									\
static void uart_zip_wbuart32_irq_config_func_##index(const struct device *dev)	\
{																\
	IRQ_CONNECT(DT_INST_IRQN(index),							\
		DT_INST_IRQ(index, priority),							\
		uart_zip_wbuart32_isr, DEVICE_DT_INST_GET(index),		\
		0);														\
	if (!DT_INST_PROP_OR(index, interrupt_no_autoenable, 0))		\
		irq_enable(DT_INST_IRQN(index));						\
}

#define UART_ZIP_WBUART32_IRQ_SETUP_FUNC(index)					\
	.irq_config_func = uart_zip_wbuart32_irq_config_func_##index

#else
#define UART_ZIP_WBUART32_IRQ_SETUP_DECL(index) /* Not used */
#define UART_ZIP_WBUART32_IRQ_SETUP(index) UART_ZIP_WBUART32_IRQ_SETUP_0(index)
#define UART_ZIP_WBUART32_IRQ_SETUP_FUNC(index) UART_ZIP_WBUART32_IRQ_SETUP_FUNC_0(index)
#endif

#define UART_ZIP_WBUART32_IRQ_SETUP_0(index) /* Not used */
#define UART_ZIP_WBUART32_IRQ_SETUP_FUNC_0(index) /* Not used */

#define UART_ZIP_WBUART32_INIT(n)												\
	UART_ZIP_WBUART32_IRQ_SETUP_DECL(n)											\
	static struct uart_zip_wbuart32_data uart_zip_wbuart32_##n##_data = {		\
		.rxErrorFlags = 0														\
	};																			\
	static const struct uart_zip_wbuart32_config uart_zip_wbuart32_##n##_cfg = {	\
		.base = DT_INST_REG_ADDR(n),												\
		.sysFreq = DT_INST_PROP(n, clock_frequency),								\
		.baudrate = DT_INST_PROP_OR(n, current_speed, 0),                           \
        .rxLglen = DT_INST_PROP(n, rx_fifo_lglen),                                  \
        .txLglen = DT_INST_PROP(n, tx_fifo_lglen),                                  \
		UART_ZIP_WBUART32_IRQ_SETUP_FUNC(n)											\
	};																				\
																					\
	DEVICE_DT_INST_DEFINE(n,														\
				&uart_zip_wbuart32_init,											\
				NULL,																\
				&uart_zip_wbuart32_##n##_data,										\
				&uart_zip_wbuart32_##n##_cfg,										\
				PRE_KERNEL_1,														\
				CONFIG_SERIAL_INIT_PRIORITY,										\
				&uart_zip_wbuart32_driver_api);										\
																					\
	UART_ZIP_WBUART32_IRQ_SETUP(n)

DT_INST_FOREACH_STATUS_OKAY(UART_ZIP_WBUART32_INIT)
