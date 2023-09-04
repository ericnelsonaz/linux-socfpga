/*
 *  Driver for Ken Crocker's SPI Journal VHDL "uart".
 *  Copyright (C) 2023 Eric Nelson <eric@nelint.com>
 *
 *  Very loosely based on drivers/tty/serial/atmel_serial.c, by Rick Bronson
 *
 */
#include <asm/io.h>
#include <asm/ioctls.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/serial.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/tty_flip.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/uaccess.h>
#include <linux/serial_core.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/suspend.h>

struct spij_uart_port {
	struct uart_port	uart;		/* uart */
	spinlock_t		lock_tx;	/* port lock */
	spinlock_t		lock_rx;	/* port lock */
	struct tasklet_struct	tasklet_rx;
	struct tasklet_struct	tasklet_tx;
	atomic_t		tasklet_shutdown;
	unsigned int		tx_len;
	struct timer_list	uart_timer;
};

static struct spij_uart_port g_spij_port;

static const struct of_device_id spij_serial_dt_ids[] = {
	{ .compatible = "kcrocker,spij-stdio" },
	{ /* sentinel */ }
};

static inline struct spij_uart_port *
to_spij_uart_port(struct uart_port *uart)
{
	return container_of(uart, struct spij_uart_port, uart);
}

static inline u32 spij_uart_readl(struct uart_port *port, u32 reg)
{
	return __raw_readl(port->membase + reg);
}

static inline void spij_uart_writel(struct uart_port *port, u32 reg, u32 value)
{
	__raw_writel(value, port->membase + reg);
}

static u_int spij_tx_empty(struct uart_port *port)
{
	pr_info("%s\n", __func__);
	return 0;
}

/*
 * Set state of the modem control output lines
 */
static void spij_set_mctrl(struct uart_port *port, u_int mctrl)
{
	pr_info("%s\n", __func__);
}

/*
 * Get state of the modem control input lines
 */
static u_int spij_get_mctrl(struct uart_port *port)
{
	pr_info("%s\n", __func__);
	return 0;
}

static void spij_stop_tx(struct uart_port *port)
{
	pr_info("%s\n", __func__);
}

static void spij_start_tx(struct uart_port *port)
{
	pr_info("%s\n", __func__);
}

static void spij_stop_rx(struct uart_port *port)
{
	pr_info("%s\n", __func__);
}

static int spij_startup(struct uart_port *port)
{
	pr_info("%s\n", __func__);
	return 0;
}

static void spij_shutdown(struct uart_port *port)
{
	pr_info("%s\n", __func__);
}

static void spij_flush_buffer(struct uart_port *port)
{
	struct spij_uart_port *spij_port = to_spij_uart_port(port);
	pr_info("%s\n", __func__);
	spij_port->tx_len = 0;
}

static void spij_set_termios(struct uart_port *port, struct ktermios *termios,
			      struct ktermios *old)
{
	pr_info("%s\n", __func__);
}

static const char *spij_type(struct uart_port *port)
{
	return "spij";
}

/*
 * Release the memory region(s) being used by 'port'.
 */
static void spij_release_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	int size = pdev->resource[0].end - pdev->resource[0].start + 1;

	release_mem_region(port->mapbase, size);

	if (port->flags & UPF_IOREMAP) {
		iounmap(port->membase);
		port->membase = NULL;
	}
}

/*
 * Request the memory region(s) being used by 'port'.
 */
static int spij_request_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	int size = pdev->resource[0].end - pdev->resource[0].start + 1;

	if (!request_mem_region(port->mapbase, size, "spij_serial"))
		return -EBUSY;

	if (port->flags & UPF_IOREMAP) {
		port->membase = ioremap(port->mapbase, size);
		if (port->membase == NULL) {
			release_mem_region(port->mapbase, size);
			return -ENOMEM;
		}
	}

	return 0;
}

static void spij_config_port(struct uart_port *port, int flags)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	int size = pdev->resource[0].end - pdev->resource[0].start + 1;

pr_info("%s: %d: %x: %u\n", __func__, flags, port->mapbase, size);

	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_ATMEL;
	
		if (!request_mem_region(port->mapbase, size, "spi_journal")) {
			pr_err("%s: error requesting memory\r\n", __func__);
			return;
		}
pr_info("%s: have memory region\n", __func__);
		if (port->flags & UPF_IOREMAP) {
			port->membase = ioremap(port->mapbase, size);
pr_info("%s: ioremap %p\n", __func__, port->membase);
			if (port->membase == NULL) {
				pr_err("%s: ioremap error\r\n", __func__);
				release_mem_region(port->mapbase, size);
			}
		} else {
pr_info("%s: ioremap not needed\n", __func__);
		}
	}

pr_info("%s: returning\n", __func__);
}

static const struct uart_ops spij_pops = {
	.tx_empty	= spij_tx_empty,
	.set_mctrl	= spij_set_mctrl,
	.get_mctrl	= spij_get_mctrl,
	.stop_tx	= spij_stop_tx,
	.start_tx	= spij_start_tx,
	.stop_rx	= spij_stop_rx,
	.startup	= spij_startup,
	.shutdown	= spij_shutdown,
	.flush_buffer	= spij_flush_buffer,
	.set_termios	= spij_set_termios,
	.type		= spij_type,
	.release_port	= spij_release_port,
	.request_port	= spij_request_port,
	.config_port	= spij_config_port,
};

/*
 * Configure the port from the platform device resource info.
 */
static int spij_init_port(struct spij_uart_port *spij_port,
				      struct platform_device *pdev)
{
	struct uart_port *port = &spij_port->uart;

	port->iotype	= UPIO_MEM;
	port->flags	= UPF_BOOT_AUTOCONF;
	port->ops	= &spij_pops;
	port->fifosize	= 1;
	port->dev	= &pdev->dev;
	port->mapbase	= pdev->resource[0].start;
	port->irq	= pdev->resource[1].start;

	return 0;
}

static struct uart_driver spij_uart = {
	.owner		= THIS_MODULE,
	.driver_name	= "spij_serial",
	.dev_name	= "spij",
	.major		= 0,
	.minor		= 0,
	.nr		= 1,
	.cons		= 0,
};

static int spij_serial_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct uart_port *port = platform_get_drvdata(pdev);

	uart_suspend_port(&spij_uart, port);

	return 0;
}

static int spij_serial_resume(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);
	uart_resume_port(&spij_uart, port);

	return 0;
}

static int spij_serial_probe(struct platform_device *pdev)
{
	struct spij_uart_port *spij_port;
	int ret = -ENODEV;

	spij_port = &g_spij_port;
	spij_port->uart.line = 0;

	atomic_set(&spij_port->tasklet_shutdown, 0);

	ret = spij_init_port(spij_port, pdev);
	if (ret)
		goto err_clear_bit;

pr_info("%s: about to add uart\n", __func__);

	ret = uart_add_one_port(&spij_uart, &spij_port->uart);
	if (ret)
		goto err_add_port;

	device_init_wakeup(&pdev->dev, 1);
	platform_set_drvdata(pdev, spij_port);

	return 0;

err_add_port:
err_clear_bit:
	return ret;
}

/*
 * Even if the driver is not modular, it makes sense to be able to
 * unbind a device: there can be many bound devices, and there are
 * situations where dynamic binding and unbinding can be useful.
 *
 * For example, a connected device can require a specific firmware update
 * protocol that needs bitbanging on IO lines, but use the regular serial
 * port in the normal case.
 */
static int spij_serial_remove(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);
	struct spij_uart_port *spij_port = to_spij_uart_port(port);
	int ret = 0;

	tasklet_kill(&spij_port->tasklet_rx);
	tasklet_kill(&spij_port->tasklet_tx);

	device_init_wakeup(&pdev->dev, 0);

	ret = uart_remove_one_port(&spij_uart, port);

	return ret;
}

static struct platform_driver spij_serial_driver = {
	.probe		= spij_serial_probe,
	.remove		= spij_serial_remove,
	.suspend	= spij_serial_suspend,
	.resume		= spij_serial_resume,
	.driver		= {
		.name			= "spij_serial",
		.of_match_table		= of_match_ptr(spij_serial_dt_ids),
	},
};

static int __init spij_serial_init(void)
{
	int ret;

	ret = uart_register_driver(&spij_uart);
	if (ret)
		return ret;

	ret = platform_driver_register(&spij_serial_driver);
	if (ret)
		uart_unregister_driver(&spij_uart);

	return ret;
}
device_initcall(spij_serial_init);
