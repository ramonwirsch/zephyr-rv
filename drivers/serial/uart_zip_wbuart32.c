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

#define TX_BUFFER_HAS_SPACE 0x00010000
#define RX_BUFFER_HAS_DATA  0x00000001
#define LGLEN_MASK          0xF
#define TX_FILL_LEVEL_SHIFT 18
#define FILL_LEVEL_MASK     0x3FF

#define UART_SETUP_REG 0x0
#define UART_STATUS_REG 0x4
#define UART_RX_REG 0x8
#define UART_TX_REG 0xC

struct uart_zip_wbuart32_config {
	uint32_t base;
	uint32_t sysFreq;
	uint32_t baudrate;
};

struct uart_zip_wbuart32_data {

};

static int uart_zip_wbuart32_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_zip_wbuart32_config *config = dev->config;
	
	uint32_t flags = sys_read32(config->base + UART_STATUS_REG);

	if (flags & RX_BUFFER_HAS_DATA) {
		*c = sys_read8(config->base + UART_RX_REG);
		return 0;
	} else {
		return -1;
	}
}

static void uart_zip_wbuart32_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_zip_wbuart32_config *config = dev->config;
	uint32_t baseAddr = config->base;

	while (!(sys_read32(baseAddr + UART_STATUS_REG) & TX_BUFFER_HAS_SPACE)) {}

	sys_write32(c, baseAddr + UART_TX_REG);
}

/*static int uart_zip_wbuart32_err_check(const struct device *dev)
{
	//const struct uart_zip_wbuart32_config *config = dev->config;
	
	return -1; //TODO not implemented
}*/

static int uart_zip_wbuart32_init(const struct device *dev)
{
	//const struct uart_zip_wbuart32_config *config = dev->config;
	
	

	return 0;
}

static const struct uart_driver_api uart_zip_wbuart32_driver_api = {
	.poll_in = uart_zip_wbuart32_poll_in,
	.poll_out = uart_zip_wbuart32_poll_out,
	.err_check = NULL
};

#define UART_ZIP_WBUART32_INIT(n)												\
	static struct uart_zip_wbuart32_data uart_zip_wbuart32_##n##_data = {		\
																				\
	};																			\
	static const struct uart_zip_wbuart32_config uart_zip_wbuart32_##n##_cfg = {	\
		.base = DT_INST_REG_ADDR(n),												\
		.sysFreq = DT_INST_PROP(n, clock_frequency),								\
		.baudrate = DT_INST_PROP_OR(n, current_speed, 0)							\
	};																				\
																					\
	DEVICE_DT_INST_DEFINE(n,														\
				&uart_zip_wbuart32_init,											\
				NULL,																\
				&uart_zip_wbuart32_##n##_data,											\
				&uart_zip_wbuart32_##n##_cfg,										\
				PRE_KERNEL_1,														\
				CONFIG_SERIAL_INIT_PRIORITY,										\
				&uart_zip_wbuart32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(UART_ZIP_WBUART32_INIT)
