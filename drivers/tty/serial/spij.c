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
#include <linux/module.h>
#include <linux/mod_devicetable.h>
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

/*==============================================================================
 * Register Macro Definitions
 *============================================================================*/
#define SPI_JOURNAL_CSR_REG				0

#define SPI_JOURNAL_CSR_TX_EN_MSK			(0x1)
#define SPI_JOURNAL_CSR_TX_EN_OFST			(0)
#define SPI_JOURNAL_CSR_TX_SCLR_MSK			(0x2)
#define SPI_JOURNAL_CSR_TX_SCLR_OFST			(1)
#define SPI_JOURNAL_CSR_RX_EN_MSK			(0x4)
#define SPI_JOURNAL_CSR_RX_EN_OFST			(2)
#define SPI_JOURNAL_CSR_RX_SCLR_MSK			(0x8)
#define SPI_JOURNAL_CSR_RX_SCLR_OFST			(3)
#define SPI_JOURNAL_CSR_BINARY_MSK			(0x10)
#define SPI_JOURNAL_CSR_BINARY_OFST			(4)
#define SPI_JOURNAL_CSR_TGT_SUPPLIES_TS_MSK		(0x20)
#define SPI_JOURNAL_CSR_TGT_SUPPLIES_TS_OFST		(5)
#define SPI_JOURNAL_CSR_FREE_RUN_CLOCK_MSK		(0x40)
#define SPI_JOURNAL_CSR_FREE_RUN_CLOCK_OFST		(6)
#define SPI_JOURNAL_CSR_MOSI_IS_SINGLE_MSK		(0x80)
#define SPI_JOURNAL_CSR_MOSI_IS_SINGLE_OFST		(7)
#define SPI_JOURNAL_CSR_CLOCK_ACTIVE_CYCLES_MSK		(0xF00)
#define SPI_JOURNAL_CSR_CLOCK_ACTIVE_CYCLES_OFST	(8)
#define SPI_JOURNAL_CSR_CLOCK_INACTIVE_CYCLES_MSK	(0xF000)
#define SPI_JOURNAL_CSR_CLOCK_INACTIVE_CYCLES_OFST	(12)
#define SPI_JOURNAL_CSR_USR_WRITES_BLOCK_MSK		(0x10000)
#define SPI_JOURNAL_CSR_USR_WRITES_BLOCK_OFST		(16)
#define SPI_JOURNAL_CSR_USR_READS_BLOCK_MSK		(0x20000)
#define SPI_JOURNAL_CSR_USR_READS_BLOCK_OFST		(17)

#define SPI_JOURNAL_INT_STAT_REG			4

#define SPI_JOURNAL_INT_STAT_STDOUT_READY_INT_MSK	(0x1)
#define SPI_JOURNAL_INT_STAT_STDOUT_READY_INT_OFST	(0)
#define SPI_JOURNAL_INT_STAT_STDIN_READY_INT_MSK	(0x2)
#define SPI_JOURNAL_INT_STAT_STDIN_READY_INT_OFST	(1)
#define SPI_JOURNAL_INT_STAT_STDOUT_OVFLW_INT_MSK	(0x4)
#define SPI_JOURNAL_INT_STAT_STDOUT_OVFLW_INT_OFST	(2)
#define SPI_JOURNAL_INT_STAT_STDIN_OVFLW_INT_MSK	(0x8)
#define SPI_JOURNAL_INT_STAT_STDIN_OVFLW_INT_OFST	(3)
#define SPI_JOURNAL_INT_STAT_USR_TX_READY_INT_MSK	(0x10)
#define SPI_JOURNAL_INT_STAT_USR_TX_READY_INT_OFST	(4)
#define SPI_JOURNAL_INT_STAT_USR_RX_READY_INT_MSK	(0x20)
#define SPI_JOURNAL_INT_STAT_USR_RX_READY_INT_OFST	(5)
#define SPI_JOURNAL_INT_STAT_USR_TX_OVFLW_INT_MSK	(0x40)
#define SPI_JOURNAL_INT_STAT_USR_TX_OVFLW_INT_OFST	(6)
#define SPI_JOURNAL_INT_STAT_USR_RX_OVFLW_INT_MSK	(0x80)
#define SPI_JOURNAL_INT_STAT_USR_RX_OVFLW_INT_OFST	(7)

#define SPI_JOURNAL_INT_ENA_REG				8

#define SPI_JOURNAL_INT_ENA_STDOUT_READY_ENA_MSK	(0x1)
#define SPI_JOURNAL_INT_ENA_STDOUT_READY_ENA_OFST	(0)
#define SPI_JOURNAL_INT_ENA_STDIN_READY_ENA_MSK		(0x2)
#define SPI_JOURNAL_INT_ENA_STDIN_READY_ENA_OFST	(1)
#define SPI_JOURNAL_INT_ENA_STDOUT_OVFLW_ENA_MSK	(0x4)
#define SPI_JOURNAL_INT_ENA_STDOUT_OVFLW_ENA_OFST	(2)
#define SPI_JOURNAL_INT_ENA_STDIN_OVFLW_ENA_MSK		(0x8)
#define SPI_JOURNAL_INT_ENA_STDIN_OVFLW_ENA_OFST	(3)
#define SPI_JOURNAL_INT_ENA_USR_TX_READY_ENA_MSK	(0x10)
#define SPI_JOURNAL_INT_ENA_USR_TX_READY_ENA_OFST	(4)
#define SPI_JOURNAL_INT_ENA_USR_RX_READY_ENA_MSK	(0x20)
#define SPI_JOURNAL_INT_ENA_USR_RX_READY_ENA_OFST	(5)
#define SPI_JOURNAL_INT_ENA_USR_TX_OVFLW_ENA_MSK	(0x40)
#define SPI_JOURNAL_INT_ENA_USR_TX_OVFLW_ENA_OFST	(6)
#define SPI_JOURNAL_INT_ENA_USR_RX_OVFLW_ENA_MSK	(0x80)
#define SPI_JOURNAL_INT_ENA_USR_RX_OVFLW_ENA_OFST	(7)

#define SPI_JOURNAL_STDIO_DATA_REG			0x0c
#define SPI_JOURNAL_STDIO_DATA_RX_Q_MSK			(0xFF)
#define SPI_JOURNAL_STDIO_DATA_RX_Q_OFST		(0)
#define SPI_JOURNAL_STDIO_DATA_TX_Q_MSK			(0xFFFFFFFF)
#define SPI_JOURNAL_STDIO_DATA_TX_Q_OFST		(0)

#define SPI_JOURNAL_USER_DATA_REG			0x10
#define SPI_JOURNAL_SP_REG				0x14

#define SPIJ_TXT_TX_RDY         0x1
#define SPIJ_TXT_RX_RDY         0x2
#define SPIJ_PKT_TX_RDY         0x3
#define SPIJ_PKT_RX_RDY         0x8

#define PKT_DRIVER_NAME "spijpkt"

static char *spijpkt_devnode(struct device *dev, umode_t *mode)
{
	if (mode)
		*mode = S_IRUGO | S_IWUGO;
	return kasprintf(GFP_KERNEL, "%s", PKT_DRIVER_NAME);
}

static int spijpkt_major;
static dev_t spijpkt_devnum;
static struct class *spijpkt_class;

struct spij_uart_port {
	struct uart_port	uart;		/* uart */
	spinlock_t		lock_tx;	/* port lock */
	spinlock_t		lock_rx;	/* port lock */
	struct tasklet_struct	tasklet_rx;
	struct tasklet_struct	tasklet_tx;
	atomic_t		tasklet_shutdown;
	unsigned int		tx_len;
	struct timer_list	uart_timer;

	/* pkt data */
	int dump_pkts;
	struct cdev cdev;
	struct device *chrdev;
	unsigned packet_length;
	wait_queue_head_t prqueue;
	wait_queue_head_t pwqueue;
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
	return readl(port->membase + reg);
}

static inline void spij_uart_writel(struct uart_port *port, u32 reg, u32 value)
{
	writel(value, port->membase + reg);
}

static inline void spij_setbits(struct uart_port *port, u32 reg, u32 mask)
{
	writel(readl(port->membase + reg) | mask, port->membase + reg);
}

static inline void spij_clrbits(struct uart_port *port, u32 reg, u32 mask)
{
	writel(readl(port->membase + reg) & ~mask, port->membase + reg);
}

static u_int spij_tx_empty(struct uart_port *port)
{
	dev_dbg(port->dev, "%s\n", __func__);

	return (spij_uart_readl(port, SPI_JOURNAL_INT_STAT_REG)
		& SPI_JOURNAL_INT_STAT_STDOUT_READY_INT_MSK)
		? TIOCSER_TEMT : 0;
}

/*
 * Set state of the modem control output lines
 */
static void spij_set_mctrl(struct uart_port *port, u_int mctrl)
{
	dev_dbg(port->dev, "%s\n", __func__);
}

/*
 * Get state of the modem control input lines
 */
static u_int spij_get_mctrl(struct uart_port *port)
{
	dev_dbg(port->dev, "%s\n", __func__);
	return 0;
}

static void spij_stop_tx(struct uart_port *port)
{
	dev_dbg(port->dev, "%s\n", __func__);
	/* disable TX ready interrupt */
	spij_clrbits(port, SPI_JOURNAL_INT_ENA_REG,
		     SPI_JOURNAL_INT_ENA_STDOUT_READY_ENA_MSK);
}

static void spij_start_tx(struct uart_port *port)
{
	dev_dbg(port->dev, "%s\n", __func__);

	/* enable TX ready interrupt */
	spij_setbits(port, SPI_JOURNAL_INT_ENA_REG,
		     SPI_JOURNAL_INT_ENA_STDOUT_READY_ENA_MSK);
}

static void spij_stop_rx(struct uart_port *port)
{
	dev_dbg(port->dev, "%s\n", __func__);
	spij_clrbits(port, SPI_JOURNAL_INT_ENA_REG,
                     SPI_JOURNAL_INT_ENA_STDIN_READY_ENA_MSK);
}

static void spij_tasklet_rx_func(unsigned long data)
{
	struct uart_port *port = (struct uart_port *)data;

	dev_dbg(port->dev, "%s\n", __func__);

	/* The interrupt handler does not take the lock */
	spin_lock(&port->lock);

	while (spij_uart_readl(port, SPI_JOURNAL_INT_STAT_REG) & SPI_JOURNAL_INT_STAT_STDIN_READY_INT_MSK) {
		u32 bytes = readl(port->membase + SPI_JOURNAL_STDIO_DATA_REG);
		while (bytes) {
			if (bytes & 0xFF) {
				u8 rx = bytes & 0xff;
				if (uart_handle_sysrq_char(port, rx))
					continue;
				if (tty_insert_flip_char(&port->state->port, rx, TTY_NORMAL) == 0)
					port->icount.buf_overrun++;
				else
					port->icount.rx++;
			}
			bytes >>= 8;
		}
	}

	spij_setbits(port, SPI_JOURNAL_INT_ENA_REG,
                     SPI_JOURNAL_INT_ENA_STDIN_READY_ENA_MSK);

	tty_flip_buffer_push(&port->state->port);
	spin_unlock(&port->lock);
}

static inline void spij_uart_write_char(struct uart_port *port, u8 value)
{
	dev_dbg(port->dev, "%s: %c\n", __func__, value);
	writel(value, port->membase + SPI_JOURNAL_STDIO_DATA_REG);
}

static void spij_uart_write_bytes(struct uart_port *port, u32 bytes)
{
	dev_dbg(port->dev, "%s: %c\n", __func__, bytes);
	writel(bytes, port->membase + SPI_JOURNAL_STDIO_DATA_REG);
}

static void spij_tx_chars(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	struct spij_uart_port *spij_port = to_spij_uart_port(port);
	u32 bytesout;
	u32 shiftout;

	dev_dbg(port->dev, "%s\n", __func__);

	if (port->x_char &&
	    (spij_uart_readl(port, SPI_JOURNAL_INT_STAT_REG) & SPI_JOURNAL_INT_STAT_STDOUT_READY_INT_MSK)) {
		spij_uart_write_char(port, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port))
		return;

	bytesout = 0;
	shiftout = 24;

	while (spij_uart_readl(port, SPI_JOURNAL_INT_STAT_REG) & SPI_JOURNAL_INT_STAT_STDOUT_READY_INT_MSK) {

		bytesout |= (xmit->buf[xmit->tail] << shiftout);
		if (0 == shiftout) {
                        spij_uart_write_bytes(port, bytesout);
			shiftout = 24;
			bytesout = 0;
		} else {
			shiftout -= 8;
		}

		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;

		if (uart_circ_empty(xmit))
			break;
	}

	if (24 != shiftout)
		spij_uart_write_bytes(port, bytesout);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (!uart_circ_empty(xmit)) {
		/* enable TX ready interrupt */
		spij_setbits(&spij_port->uart, SPI_JOURNAL_INT_ENA_REG,
			     SPI_JOURNAL_INT_ENA_STDOUT_READY_ENA_MSK);
	}
}

static void spij_tasklet_tx_func(unsigned long data)
{
	struct uart_port *port = (struct uart_port *)data;

	/* The interrupt handler does not take the lock */
	spin_lock(&port->lock);
        spij_tx_chars(port);
	spin_unlock(&port->lock);
}

static int spij_startup(struct uart_port *port)
{
	struct spij_uart_port *spij_port = to_spij_uart_port(port);
	dev_dbg(port->dev, "%s\n", __func__);
	spij_setbits(&spij_port->uart, SPI_JOURNAL_INT_ENA_REG,
                     SPI_JOURNAL_INT_ENA_STDIN_READY_ENA_MSK
		     |SPI_JOURNAL_INT_ENA_STDOUT_OVFLW_ENA_MSK
		     |SPI_JOURNAL_INT_ENA_STDIN_OVFLW_ENA_MSK
		     |SPI_JOURNAL_INT_ENA_USR_TX_OVFLW_ENA_MSK
		     |SPI_JOURNAL_INT_ENA_USR_RX_OVFLW_ENA_MSK);

	tasklet_init(&spij_port->tasklet_rx, spij_tasklet_rx_func,
			(unsigned long)port);
	tasklet_init(&spij_port->tasklet_tx, spij_tasklet_tx_func,
			(unsigned long)port);
	return 0;
}

static void spij_shutdown(struct uart_port *port)
{
	dev_dbg(port->dev, "%s\n", __func__);
}

static void spij_flush_buffer(struct uart_port *port)
{
	struct spij_uart_port *spij_port = to_spij_uart_port(port);
	dev_dbg(port->dev, "%s\n", __func__);
	spij_port->tx_len = 0;
}

static void spij_set_termios(struct uart_port *port, struct ktermios *termios,
			      struct ktermios *old)
{
	dev_dbg(port->dev, "%s\n", __func__);
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
	struct resource *res;
	void __iomem *base;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base)) {
		pr_err("%s: error mapping 0x%x\n", __func__, res->start);
		return PTR_ERR(base);
	}

	dev_dbg(port->dev, "%s: map 0x%x to 0x%x\n", __func__, res->start, (unsigned)base);
	port->mapbase = res->start;
	port->membase = base;
	port->iotype = UPIO_MEM;

	return 0;
}

static void spij_config_port(struct uart_port *port, int flags)
{
	dev_dbg(port->dev, "%s: %d: %x\n", __func__, flags, port->mapbase);

	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_ATMEL;
		spij_request_port(port);
	}
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
static int spij_init_port
	(struct spij_uart_port *spij_port,
	 struct platform_device *pdev)
{
	struct uart_port *port = &spij_port->uart;

	port->iotype	= UPIO_MEM;
	port->flags	= UPF_BOOT_AUTOCONF;
	port->ops	= &spij_pops;
	port->fifosize	= 1;
	port->dev	= &pdev->dev;
	port->mapbase	= pdev->resource[0].start;
        port->irq	= platform_get_irq(pdev, 0);

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

static irqreturn_t spij_int(int irq, void *dev_id)
{
	struct spij_uart_port *spij_port = dev_id;
	u32 stat = spij_uart_readl(&spij_port->uart, SPI_JOURNAL_INT_STAT_REG);

	dev_dbg(spij_port->uart.dev, "%s: 0x08%x\n", __func__, stat);

	if (stat & SPI_JOURNAL_INT_STAT_STDOUT_READY_INT_MSK) {
                tasklet_schedule(&spij_port->tasklet_tx);
		spij_clrbits(&spij_port->uart, SPI_JOURNAL_INT_ENA_REG,
			     SPI_JOURNAL_INT_STAT_STDOUT_READY_INT_MSK);
	}

	if (stat & SPI_JOURNAL_INT_STAT_STDIN_READY_INT_MSK) {
                tasklet_schedule(&spij_port->tasklet_rx);
		spij_clrbits(&spij_port->uart, SPI_JOURNAL_INT_ENA_REG,
			     SPI_JOURNAL_INT_STAT_STDIN_READY_INT_MSK);
	}

	if (stat & SPI_JOURNAL_INT_STAT_STDOUT_OVFLW_INT_MSK) {
		dev_info(spij_port->uart.dev, "tx ovfl\n");
		spij_clrbits(&spij_port->uart, SPI_JOURNAL_INT_STAT_REG,
                             SPI_JOURNAL_INT_STAT_STDOUT_OVFLW_INT_MSK);
	}

	if (stat & SPI_JOURNAL_INT_STAT_STDIN_OVFLW_INT_MSK) {
		dev_dbg(spij_port->uart.dev, "rx ovfl\n");
		spij_clrbits(&spij_port->uart, SPI_JOURNAL_INT_STAT_REG,
                             SPI_JOURNAL_INT_STAT_STDIN_OVFLW_INT_MSK);
		spij_port->uart.icount.buf_overrun++;
	}

	if (stat & SPI_JOURNAL_INT_STAT_USR_TX_READY_INT_MSK) {
		spij_clrbits(&spij_port->uart, SPI_JOURNAL_INT_ENA_REG,
			     SPI_JOURNAL_INT_ENA_USR_TX_READY_ENA_MSK);
		wake_up(&spij_port->pwqueue);
	}

	if (stat & SPI_JOURNAL_INT_STAT_USR_RX_READY_INT_MSK) {
		spij_clrbits(&spij_port->uart, SPI_JOURNAL_INT_ENA_REG,
			     SPI_JOURNAL_INT_ENA_USR_RX_READY_ENA_MSK);
		wake_up(&spij_port->prqueue);
	}

	if (stat & SPI_JOURNAL_INT_STAT_USR_TX_OVFLW_INT_MSK) {
		dev_info(spij_port->uart.dev, "pkt tx ovfl\n");
		spij_clrbits(&spij_port->uart, SPI_JOURNAL_INT_STAT_REG,
                             SPI_JOURNAL_INT_STAT_USR_TX_OVFLW_INT_MSK);
	}

	if (stat & SPI_JOURNAL_INT_STAT_USR_RX_OVFLW_INT_MSK) {
		dev_info(spij_port->uart.dev, "pkt rx ovfl\n");
		spij_clrbits(&spij_port->uart, SPI_JOURNAL_INT_STAT_REG,
                             SPI_JOURNAL_INT_STAT_USR_RX_OVFLW_INT_MSK);
	}

	return IRQ_HANDLED;
}

static inline int pkt_rx_rdy(struct uart_port *port)
{
	return 0 != (spij_uart_readl(port, SPI_JOURNAL_INT_STAT_REG)
		        & SPI_JOURNAL_INT_STAT_USR_RX_READY_INT_MSK);
}

static ssize_t pkt_read
	(struct file *file, char __user *buf,
	 size_t count, loff_t *ppos)
{
	struct spij_uart_port *sport = (struct spij_uart_port *)file->private_data;
	struct uart_port *port = &sport->uart;
	u32 longwords[3];
	int retval;

	if (count != sport->packet_length)
		return -EINVAL;

	if (!pkt_rx_rdy(port) && !(file->f_flags & O_NONBLOCK)) {
		spij_setbits(port, SPI_JOURNAL_INT_ENA_REG,
			     SPI_JOURNAL_INT_ENA_USR_RX_READY_ENA_MSK);
		wait_event_interruptible(sport->prqueue, pkt_rx_rdy(port));
	}

	spin_lock(&port->lock);

	if (pkt_rx_rdy(port)) {
		longwords[0] = spij_uart_readl(port, SPI_JOURNAL_USER_DATA_REG);
		longwords[1] = spij_uart_readl(port, SPI_JOURNAL_USER_DATA_REG);
		if (12 == count)
			longwords[2] = spij_uart_readl(port, SPI_JOURNAL_USER_DATA_REG);
		else
			longwords[2] = 0;

		if (sport->dump_pkts) {
			if (8 == count)
				dev_info(sport->chrdev, "-> 0x%08x 0x%08x\n",
					 longwords[0], longwords[1]);
			else
				dev_info(sport->chrdev, "-> 0x%08x 0x%08x 0x%08x\n",
					 longwords[0], longwords[1], longwords[2]);
		}
		retval = count;
		*ppos += retval;
	} else {
		retval = -EINTR;
	}

	spin_unlock(&port->lock);

	return retval;
}

static inline int pkt_tx_rdy(struct uart_port *port)
{
	return 0 != (spij_uart_readl(port, SPI_JOURNAL_INT_STAT_REG)
		        & SPI_JOURNAL_INT_STAT_USR_TX_READY_INT_MSK);
}

static ssize_t pkt_write
	(struct file *file, const char __user *data, size_t len, loff_t *offs)
{
	struct spij_uart_port *sport = (struct spij_uart_port *)file->private_data;
	struct uart_port *port = &sport->uart;
	u32 longwords[4];
	int retval;

	if (len != sport->packet_length)
		return -EINVAL;

	if (copy_from_user(longwords, data, len))
		return -EFAULT;

	if (!pkt_tx_rdy(port) && !(file->f_flags & O_NONBLOCK)) {
		spij_setbits(port, SPI_JOURNAL_INT_ENA_REG,
			     SPI_JOURNAL_INT_ENA_USR_TX_READY_ENA_MSK);
		wait_event_interruptible(sport->pwqueue, pkt_tx_rdy(port));
	}

	spin_lock(&port->lock);

	if (pkt_tx_rdy(port)) {
		spij_uart_writel(port, SPI_JOURNAL_USER_DATA_REG, longwords[0]);
		spij_uart_writel(port, SPI_JOURNAL_USER_DATA_REG, longwords[1]);
		if (len == 12)
			spij_uart_writel(port, SPI_JOURNAL_USER_DATA_REG, longwords[2]);

		retval = len;
		if (sport->dump_pkts) {
			if (8 == len)
				dev_info(sport->chrdev, "-> 0x%08x 0x%08x\n",
					 longwords[0], longwords[1]);
			else
				dev_info(sport->chrdev, "-> 0x%08x 0x%08x 0x%08x\n",
					 longwords[0], longwords[1], longwords[2]);
		}
	} else {
		retval = -EINTR;
	}

	spin_unlock(&port->lock);

	return retval;
}

static unsigned int pkt_poll(struct file *file, struct poll_table_struct *table)
{
	struct spij_uart_port *sport = (struct spij_uart_port *)file->private_data;;

	dev_dbg(sport->chrdev, "%s\n", __func__);
	return 0;
}

static int pkt_open(struct inode *inode, struct file *file)
{
	struct spij_uart_port *sport;

	sport = container_of(inode->i_cdev, struct spij_uart_port, cdev);
	dev_dbg(sport->chrdev, "%s: %px/%px\n", __func__, sport, &g_spij_port);
	file->private_data = sport;

	return 0;
}

static int pkt_release(struct inode *inode, struct file *file)
{
	struct spij_uart_port *sport;

	sport = container_of(inode->i_cdev, struct spij_uart_port, cdev);
	dev_dbg(sport->chrdev, "%s\n", __func__);
	return 0;
}

static struct file_operations const pkt_fops = {
	.owner = THIS_MODULE,
	.read = pkt_read,
	.write = pkt_write,
	.poll = pkt_poll,
	.open = pkt_open,
	.release = pkt_release,
};

static ssize_t spij_dump_read
	(struct device *dev,
	 struct device_attribute *attr,
	 char *buf)
{
	return sprintf(buf, "%d", g_spij_port.dump_pkts);
}

static ssize_t spij_dump_write
	(struct device *dev,
	 struct device_attribute *attr,
	 const char *buf, size_t count)
{
	unsigned value;
	int err;

	err = kstrtouint(buf, 10, &value);
	if (err) {
		dev_err(dev, "%s: invalid value %d:%s\n", __func__, err, buf);
		return -EINVAL;
	}

	g_spij_port.dump_pkts = value;

	return count;
}

static DEVICE_ATTR(dump, S_IWUSR | S_IRUGO,
		   spij_dump_read,
		   spij_dump_write);

static int spij_serial_probe(struct platform_device *pdev)
{
	struct spij_uart_port *spij_port;
	int ret = -ENODEV;

	spij_port = &g_spij_port;
	spij_port->uart.line = 0;
	init_waitqueue_head(&spij_port->prqueue);
	init_waitqueue_head(&spij_port->pwqueue);
	spij_port->packet_length = 8;

	atomic_set(&spij_port->tasklet_shutdown, 0);

	ret = spij_init_port(spij_port, pdev);
	if (ret)
		goto out;

	ret = uart_add_one_port(&spij_uart, &spij_port->uart);
	if (ret)
		goto out;

	device_init_wakeup(&pdev->dev, 1);

	ret = devm_request_irq(&pdev->dev, spij_port->uart.irq,
			       spij_int, 0, dev_name(&pdev->dev),
			       spij_port);
	if (ret) {
		dev_err(&pdev->dev, "%s: error %d requesting irq\n",
			__func__, ret);
		goto out;
	}

	cdev_init(&spij_port->cdev, &pkt_fops);
	spij_port->cdev.owner = THIS_MODULE;
	spij_port->cdev.ops = &pkt_fops;

	ret = cdev_add(&spij_port->cdev, spijpkt_devnum, 1);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: couldn't add device: err %d\n",
			PKT_DRIVER_NAME, ret);
		goto out;
	}

	spij_port->chrdev = device_create(spijpkt_class, &platform_bus,
					  spijpkt_devnum, 0, "%s",
					  PKT_DRIVER_NAME);
	platform_set_drvdata(pdev, spij_port);

	if (spij_uart_readl(&spij_port->uart, SPI_JOURNAL_CSR_REG) & SPI_JOURNAL_CSR_TGT_SUPPLIES_TS_MSK)
		spij_port->packet_length += 4;

	pr_info("%s: spijpkt device created\n", __func__);

	ret = device_create_file(&pdev->dev, &dev_attr_dump);
	dev_info(&pdev->dev, "%s: dump sysfs %d\n", __func__, ret);

out:
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

	spijpkt_class = class_create(THIS_MODULE, "spijpkt");
	if (IS_ERR(spijpkt_class)) {
		ret = PTR_ERR(spijpkt_class);
		goto out;
	}

	spijpkt_class->devnode = spijpkt_devnode;
	ret = alloc_chrdev_region(&spijpkt_devnum, 0, 1, PKT_DRIVER_NAME);
	if (ret < 0) {
		pr_err("%s: couldn't alloc chrdevs: err %d\n",
		       PKT_DRIVER_NAME, ret);
		goto fail_chrdev;
	}
	spijpkt_major = MAJOR(spijpkt_devnum);

	pr_info("spijpkt class created\n");

	ret = uart_register_driver(&spij_uart);
	if (ret)
		goto fail_platform;

	ret = platform_driver_register(&spij_serial_driver);
	if (ret)
		goto fail_class;

	goto out;

fail_class:
	uart_unregister_driver(&spij_uart);

fail_platform:
	unregister_chrdev_region(spijpkt_devnum, 1);

fail_chrdev:
	class_destroy(spijpkt_class);
	spijpkt_class = 0;
out:
	return ret;
}

device_initcall(spij_serial_init);
MODULE_LICENSE("GPL");
