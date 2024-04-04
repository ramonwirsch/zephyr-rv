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
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	uart_irq_config_func_t irq_config_func;
	int16_t irq_idx;
#endif
};

struct uart_zip_wbuart_tx_queue {
	const uint8_t* orgPtr;
	uint16_t orgBytes;
	uint16_t sentBytes;
};

struct uart_zip_wbuart32_data {
    uint32_t fifoStatCache;
    uint32_t rxErrorFlags;
    uint16_t maxTxBuf;
    uint16_t maxRxBuf;
    uint8_t irqMask;

#ifdef CONFIG_UART_ASYNC_API
    void (*async_callback)(const struct device *dev, struct uart_event *evt, void *user_data);
	void* async_user;
    struct uart_zip_wbuart_tx_queue txState;
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

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)

static struct uart_event asyncEvtHolder;

static void uart_zip_wbuart32_async_sendTxDone(const struct device *dev, struct uart_zip_wbuart32_data *data, const uint8_t* buf, uint16_t len) {
	if (data->async_callback) {
		asyncEvtHolder.type = UART_TX_DONE;
		asyncEvtHolder.data.tx.buf = buf;
		asyncEvtHolder.data.tx.len = len;
		data->async_callback(dev, &asyncEvtHolder, data->async_user);
	}
}

static void uart_zip_wbuart32_isr(const struct device *dev) {
	struct uart_zip_wbuart32_data *data = dev->data;
	const struct uart_zip_wbuart32_config *config = dev->config;

#ifdef CONFIG_UART_ASYNC_API
	uint16_t orgTxLen = data->txState.orgBytes;
	if (orgTxLen) {
		uint16_t sentTxLen = data->txState.sentBytes;
		uint16_t remLen = orgTxLen-sentTxLen;
		int nextSent = uart_zip_wbuart32_fifo_fill(dev, data->txState.orgPtr+sentTxLen, remLen);
		uint16_t totalSentLen = sentTxLen + nextSent;
		if (totalSentLen == orgTxLen) { // done
			data->txState.orgBytes = 0;
			uart_zip_wbuart32_interrupt_enable_masked(config->base, data, 0, UART_ZIP_WBUART_IRQ_TX_HALF | UART_ZIP_WBUART_IRQ_TX_SPACE);
			const uint8_t* ptr = data->txState.orgPtr;
			data->txState.orgPtr = NULL;
			data->txState.sentBytes = 0;
			
			uart_zip_wbuart32_async_sendTxDone(dev, data, ptr, totalSentLen);
		} else {
			data->txState.sentBytes = totalSentLen;
		}
	}
#endif
}

int uart_zip_wbuart32_async_tx(const struct device *dev, const uint8_t *buf, size_t len, int32_t timeout) {
	struct uart_zip_wbuart32_data *data = dev->data;
	const struct uart_zip_wbuart32_config *config = dev->config;
	ARG_UNUSED(timeout); //TODO

	if (data->txState.orgBytes) {
		return -EBUSY;
	}

	int alreadyDone = uart_zip_wbuart32_fifo_fill(dev, buf, len);

	if (alreadyDone >= len) {
		uart_zip_wbuart32_async_sendTxDone(dev, data, buf, len);
		return 0;
	} else {
		data->txState.orgPtr = buf;
		data->txState.orgBytes = len;
		data->txState.sentBytes = alreadyDone;
		
		uart_zip_wbuart32_interrupt_enable_masked(config->base, data, UART_ZIP_WBUART_IRQ_TX_HALF, UART_ZIP_WBUART_IRQ_TX_HALF | UART_ZIP_WBUART_IRQ_TX_SPACE);
		return 0;
	}
}

int uart_zip_wbuart32_async_tx_abort(const struct device *dev) {
	struct uart_zip_wbuart32_data *data = dev->data;
	const struct uart_zip_wbuart32_config *config = dev->config;

	if (data->txState.orgBytes) { // ongoing transmission
		uart_zip_wbuart32_interrupt_enable_masked(config->base, data, 0, UART_ZIP_WBUART_IRQ_TX_HALF | UART_ZIP_WBUART_IRQ_TX_SPACE);
		data->txState.orgBytes = 0;
		const uint8_t* ptr = data->txState.orgPtr;
		uint16_t sent = data->txState.sentBytes;
		data->txState.orgPtr = NULL;
		data->txState.sentBytes = 0;
		uart_zip_wbuart32_async_sendTxDone(dev, data, ptr, sent);
		return 0;
	} else {
		return -EFAULT;
	}
}

int uart_zip_wbuart32_async_callback_set(const struct device *dev, uart_callback_t callback, void *user_data) {
	struct uart_zip_wbuart32_data *data = dev->data;
	const struct uart_zip_wbuart32_config *config = dev->config;

	if (config->irq_config_func) {

		data->async_callback = callback;
		data->async_user = user_data;
	}

	return 0;
}

#endif

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

	uart_zip_wbuart32_interrupt_enable_masked(config->base, data, 0, 0xFF);

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	if (config->irq_config_func) {
		config->irq_config_func(dev); // calls IRQ_CONNECT for the ISR-tables, does not yet enable the interrupt
	}
#endif

	return 0;
}

static const struct uart_driver_api uart_zip_wbuart32_driver_api = {
	.poll_in = uart_zip_wbuart32_poll_in,
	.poll_out = uart_zip_wbuart32_poll_out,
	.err_check = uart_zip_wbuart32_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_zip_wbuart32_fifo_fill,
	.fifo_read = uart_zip_wbuart32_fifo_read,
	.irq_tx_enable = NULL,
	.irq_tx_disable = NULL,
	.irq_tx_ready = uart_zip_wbuart32_tx_ready,
	.irq_rx_enable = NULL,
	.irq_rx_disable = NULL,
	.irq_tx_complete = uart_zip_wbuart32_tx_complete, // works without actually using interrupts, but structs must still be compiled for it
	.irq_rx_ready = uart_zip_wbuart32_rx_ready,
	.irq_err_enable = NULL,
	.irq_err_disable = NULL,
	.irq_is_pending = NULL,
	.irq_update = NULL,
	.irq_callback_set = NULL,
#endif
#ifdef CONFIG_UART_ASYNC_API
	.callback_set = uart_zip_wbuart32_async_callback_set,
	.tx = uart_zip_wbuart32_async_tx,
	.tx_abort = uart_zip_wbuart32_async_tx_abort,
	.rx_enable = NULL,
	.rx_buf_rsp = NULL,
	.rx_disable = NULL,
#endif
};

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
#define UART_ZIP_WBUART32_IRQ_SETUP_DECL(index)				\
	static void uart_zip_wbuart32_irq_config_func_##index(const struct device *dev);

#define UART_ZIP_WBUART32_IRQ_SETUP_IMPL_CONNECT(index)									\
static void uart_zip_wbuart32_irq_config_func_##index(const struct device *dev)	{ \
	IRQ_CONNECT(DT_INST_IRQN(index),							\
		DT_INST_IRQ(index, priority),							\
		uart_zip_wbuart32_isr, DEVICE_DT_INST_GET(index),		\
		0);														\
	irq_enable(DT_INST_IRQN(index));							\
}

#define UART_ZIP_WBUART32_IRQ_SETUP_IMPL_NO_CONNECT(index)									\
static void uart_zip_wbuart32_irq_config_func_##index(const struct device *dev)	{}

#define UART_ZIP_WBUART32_IRQ_SETUP_IMPL(index) COND_CODE_0(DT_INST_PROP_OR(index, interrupt_no_connect_driver, 0), (UART_ZIP_WBUART32_IRQ_SETUP_IMPL_CONNECT), (UART_ZIP_WBUART32_IRQ_SETUP_IMPL_NO_CONNECT))(index)

#define UART_ZIP_WBUART32_IRQ_SETUP(index)					\
	.irq_config_func = DT_INST_PROP_OR(index, interrupt_no_connect_driver, 0)? NULL : uart_zip_wbuart32_irq_config_func_##index

#else
#define UART_ZIP_WBUART32_IRQ_SETUP_DECL(index) /* Not used */
#define UART_ZIP_WBUART32_IRQ_SETUP_IMPL(index) /* Not used */
#define UART_ZIP_WBUART32_IRQ_SETUP(index) /* Not used */
#endif

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
		UART_ZIP_WBUART32_IRQ_SETUP(n)											\
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
	UART_ZIP_WBUART32_IRQ_SETUP_IMPL(n)

DT_INST_FOREACH_STATUS_OKAY(UART_ZIP_WBUART32_INIT)
