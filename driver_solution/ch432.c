// SPDX-License-Identifier: GPL-2.0
/*
 * CH432 SPI to RS485/RS422 driver for Linux - Optimized for Modbus RTU
 * 
 * Based on working polling implementation from ch432_polling_based.c
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/kfifo.h>
#include <linux/delay.h>

#define DRIVER_NAME "ch432_spi"
#define TTY_DEVICE_NAME "ttyWCH"
#define MAX_PORTS 2

// CH432T register definitions
#define CH432T_RBR_REG 0x00
#define CH432T_THR_REG 0x00
#define CH432T_IER_REG 0x01
#define CH432T_IIR_REG 0x02
#define CH432T_FCR_REG 0x02
#define CH432T_LCR_REG 0x03
#define CH432T_MCR_REG 0x04
#define CH432T_LSR_REG 0x05
#define CH432T_MSR_REG 0x06
#define CH432T_SCR_REG 0x07
#define CH432T_DLL_REG 0x00
#define CH432T_DLH_REG 0x01

// Register bit definitions
#define CH432T_IER_RDI_BIT   BIT(0)
#define CH432T_IER_THRI_BIT  BIT(1)
#define CH432T_IER_RLSI_BIT  BIT(2)
#define CH432T_IER_MSI_BIT   BIT(3)
#define CH432T_IER_CK2X_BIT  BIT(5)
#define CH432T_IER_SLEEP_BIT BIT(5)
#define CH432T_IER_LOWPOWER_BIT BIT(6)
#define CH432T_IER_RESET_BIT BIT(7)

#define CH432T_FCR_FIFO_BIT    BIT(0)
#define CH432T_FCR_RXRESET_BIT BIT(1)
#define CH432T_FCR_TXRESET_BIT BIT(2)
#define CH432T_FCR_RXLVL_BIT   GENMASK(7, 6)

#define CH432T_LCR_DLAB_BIT    BIT(7)

#define CH432T_MCR_RTS_BIT     BIT(1)
#define CH432T_MCR_AFE         BIT(5)

#define CH432T_LSR_DR_BIT      BIT(0)
#define CH432T_LSR_THRE_BIT    BIT(5)

#define CH432T_CLOCK_FREQUENCY 22118400

struct ch432_port {
    struct spi_device *spi;
    struct tty_port port;
    struct kfifo rx_fifo;
    struct kfifo tx_fifo;
    int gpio_int;
    int irq;
    bool open;
    u8 portnum;  // 0 or 1 for CH432T_PORT_1/CH432T_PORT_2
};

static struct tty_driver *ch432_tty_driver;
static struct ch432_port *ch432_ports[MAX_PORTS];

// TTY port operations
static int ch432_port_activate(struct tty_port *port, struct tty_struct *tty)
{
    return 0;
}

static void ch432_port_shutdown(struct tty_port *port)
{
    // Cleanup when port is closed
}

static const struct tty_port_operations ch432_port_ops = {
    .activate = ch432_port_activate,
    .shutdown = ch432_port_shutdown,
};

// SPI register access functions with proper delays
static u8 ch432_read_reg(struct ch432_port *priv, u8 reg)
{
    u8 tx_buf[2] = {0};
    u8 rx_buf[2] = {0};
    struct spi_transfer xfer = {
        .tx_buf = tx_buf,
        .rx_buf = rx_buf,
        .len = 2,
        .delay = {
            .value = 1,
            .unit = SPI_DELAY_UNIT_USECS,
        },
    };
    struct spi_message msg;

    // Format read command: 0xFD & ((reg + portnum * 0x08) << 2)
    tx_buf[0] = 0xFD & ((reg + priv->portnum * 0x08) << 2);
    tx_buf[1] = 0x00; // Dummy byte for read

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    spi_sync(priv->spi, &msg);
    
    // Additional delay as per polling implementation
    udelay(500);
    
    return rx_buf[1];
}

static void ch432_write_reg(struct ch432_port *priv, u8 reg, u8 val)
{
    u8 tx_buf[2] = {0};
    struct spi_transfer xfer = {
        .tx_buf = tx_buf,
        .len = 2,
        .delay = {
            .value = 1,
            .unit = SPI_DELAY_UNIT_USECS,
        },
    };
    struct spi_message msg;

    // Format write command: 0x02 | ((reg + portnum * 0x08) << 2)
    tx_buf[0] = 0x02 | ((reg + priv->portnum * 0x08) << 2);
    tx_buf[1] = val;

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    spi_sync(priv->spi, &msg);
    
    // Additional delay as per polling implementation
    udelay(500);
}

// Hardware initialization with proper configuration sequence
static void ch432_init_port(struct ch432_port *priv)
{
    // Test communication with scratch register
    ch432_write_reg(priv, CH432T_SCR_REG, 0x66);
    udelay(2000); // 2ms delay as in polling code
    u8 scr = ch432_read_reg(priv, CH432T_SCR_REG);
    printk(KERN_INFO "CH432 SCR register test: wrote 0x66, read 0x%02x\n", scr);

    // Reset FIFOs
    ch432_write_reg(priv, CH432T_FCR_REG, 
                   CH432T_FCR_FIFO_BIT | 
                   CH432T_FCR_RXRESET_BIT | 
                   CH432T_FCR_TXRESET_BIT);

    // Default configuration: 9600 8N1 (Modbus default)
    u8 lcr = (0x03 << 0); // 8 data bits
    ch432_write_reg(priv, CH432T_LCR_REG, lcr);

    // Set baud rate (Modbus typically uses 9600, 19200, 38400, etc.)
    u32 baud = 9600;
    u32 divisor = CH432T_CLOCK_FREQUENCY / 16 / baud;
    
    // Access divisor latch
    ch432_write_reg(priv, CH432T_LCR_REG, lcr | CH432T_LCR_DLAB_BIT);
    udelay(2000); // 2ms delay as in polling code
    
    // Set divisor
    ch432_write_reg(priv, CH432T_DLL_REG, divisor & 0xFF);
    ch432_write_reg(priv, CH432T_DLH_REG, (divisor >> 8) & 0xFF);
    
    // Restore LCR
    ch432_write_reg(priv, CH432T_LCR_REG, lcr);

    // Enable RTS, AFE for RS485
    ch432_write_reg(priv, CH432T_MCR_REG, CH432T_MCR_RTS_BIT | CH432T_MCR_AFE);

    // Disable interrupts (we'll use polling for Modbus)
    ch432_write_reg(priv, CH432T_IER_REG, 0);
}

// Interrupt service routine - simplified for Modbus
static irqreturn_t ch432_interrupt(int irq, void *dev_id)
{
    struct ch432_port *priv = dev_id;
    u8 iir, lsr;
    int processed = 0;

    do {
        iir = ch432_read_reg(priv, CH432T_IIR_REG);
        if (iir & 0x01) // No interrupt pending
            break;

        switch ((iir >> 1) & 0x07) {
        case 0x04: // Received data available
            lsr = ch432_read_reg(priv, CH432T_LSR_REG);
            while (lsr & CH432T_LSR_DR_BIT) {
                u8 ch = ch432_read_reg(priv, CH432T_RBR_REG);
                if (!kfifo_put(&priv->rx_fifo, ch)) {
                    break; // FIFO full
                }
                lsr = ch432_read_reg(priv, CH432T_LSR_REG);
            }
            tty_flip_buffer_push(&priv->port);
            break;

        default:
            break;
        }

        processed++;
    } while (processed < 5); // Limit number of interrupts processed

    return IRQ_HANDLED;
}

// TTY operations optimized for Modbus
static int ch432_tty_open(struct tty_struct *tty, struct file *filp)
{
    struct ch432_port *priv = ch432_ports[tty->index];
    
    if (!priv)
        return -ENODEV;

    tty->driver_data = priv;
    priv->open = true;
    
    return tty_port_open(&priv->port, tty, filp);
}

static void ch432_tty_close(struct tty_struct *tty, struct file *filp)
{
    struct ch432_port *priv = tty->driver_data;
    
    if (priv) {
        tty_port_close(&priv->port, tty, filp);
        priv->open = false;
    }
}

static ssize_t ch432_tty_write(struct tty_struct *tty, const u8 *buf, size_t count)
{
    struct ch432_port *priv = tty->driver_data;
    int ret;
    size_t i;

    if (!priv)
        return -ENODEV;

    // Direct write for Modbus - don't use FIFO to maintain timing
    for (i = 0; i < count; i++) {
        // Wait until transmitter is ready (as in polling code)
        int timeout = 1000; // 1ms timeout
        while (!(ch432_read_reg(priv, CH432T_LSR_REG) & CH432T_LSR_THRE_BIT)) {
            if (--timeout <= 0)
                return -ETIMEDOUT;
            udelay(1);
        }
        ch432_write_reg(priv, CH432T_THR_REG, buf[i]);
    }

    return count;
}

static unsigned int ch432_tty_write_room(struct tty_struct *tty)
{
    // For Modbus, we can always accept data (flow control is handled elsewhere)
    return 4096;
}

static unsigned int ch432_tty_chars_in_buffer(struct tty_struct *tty)
{
    // For Modbus, we don't buffer in software
    return 0;
}

static void ch432_tty_set_termios(struct tty_struct *tty, const struct ktermios *old_termios)
{
    struct ch432_port *priv = tty->driver_data;
    u8 lcr = 0;
    u32 baud;

    if (!priv)
        return;

    // Set data bits (Modbus typically uses 8)
    switch (tty->termios.c_cflag & CSIZE) {
    case CS5: lcr |= 0x00; break;
    case CS6: lcr |= 0x01; break;
    case CS7: lcr |= 0x02; break;
    case CS8: lcr |= 0x03; break;
    }

    // Set stop bits (Modbus typically uses 1 or 2)
    if (tty->termios.c_cflag & CSTOPB)
        lcr |= 0x04;

    // Set parity (Modbus typically uses none or even)
    if (tty->termios.c_cflag & PARENB) {
        lcr |= 0x08;
        if (!(tty->termios.c_cflag & PARODD))
            lcr |= 0x10; // Even parity
    }

    // Set new LCR
    ch432_write_reg(priv, CH432T_LCR_REG, lcr);

    // Set baud rate
    baud = tty_get_baud_rate(tty);
    if (baud == 0)
        baud = 9600;

    u32 divisor = CH432T_CLOCK_FREQUENCY / 16 / baud;
    
    // Access divisor latch
    ch432_write_reg(priv, CH432T_LCR_REG, lcr | CH432T_LCR_DLAB_BIT);
    udelay(2000); // 2ms delay
    
    // Set divisor
    ch432_write_reg(priv, CH432T_DLL_REG, divisor & 0xFF);
    ch432_write_reg(priv, CH432T_DLH_REG, (divisor >> 8) & 0xFF);
    
    // Restore LCR
    ch432_write_reg(priv, CH432T_LCR_REG, lcr);
}

static const struct tty_operations ch432_tty_ops = {
    .open = ch432_tty_open,
    .close = ch432_tty_close,
    .write = ch432_tty_write,
    .write_room = ch432_tty_write_room,
    .chars_in_buffer = ch432_tty_chars_in_buffer,
    .set_termios = ch432_tty_set_termios,
};

// SPI driver functions
static int ch432_probe(struct spi_device *spi)
{
    struct device *dev = &spi->dev;
    struct ch432_port *priv;
    int ret, index;
    u32 val;

    // Configure SPI - MODE 0, 50kHz as in polling code
    spi->mode = SPI_MODE_0;
    spi->max_speed_hz = 50000;
    ret = spi_setup(spi);
    if (ret < 0) {
        dev_err(dev, "Failed to setup SPI: %d\n", ret);
        return ret;
    }

    // Get port index from DT alias or use 0 as default
    if (of_property_read_u32(dev->of_node, "port-number", &val) == 0) {
        index = val;
    } else {
        index = 0;
    }

    if (index >= MAX_PORTS) {
        dev_err(dev, "Invalid port number %d\n", index);
        return -EINVAL;
    }

    priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;

    spi_set_drvdata(spi, priv);
    priv->spi = spi;
    priv->portnum = index;

    // Initialize FIFOs
    ret = kfifo_alloc(&priv->rx_fifo, 4096, GFP_KERNEL);
    if (ret) {
        dev_err(dev, "Failed to allocate RX FIFO\n");
        return ret;
    }

    ret = kfifo_alloc(&priv->tx_fifo, 4096, GFP_KERNEL);
    if (ret) {
        dev_err(dev, "Failed to allocate TX FIFO\n");
        kfifo_free(&priv->rx_fifo);
        return ret;
    }

    // Initialize TTY port
    tty_port_init(&priv->port);
    priv->port.ops = &ch432_port_ops;

    // Get interrupt GPIO (optional for Modbus)
    priv->gpio_int = of_get_named_gpio(dev->of_node, "interrupt-gpios", 0);
    if (gpio_is_valid(priv->gpio_int)) {
        ret = devm_gpio_request(dev, priv->gpio_int, "ch432_irq");
        if (ret) {
            dev_err(dev, "Failed to request GPIO %d\n", priv->gpio_int);
            goto err_free_fifos;
        }

        ret = gpio_direction_input(priv->gpio_int);
        if (ret) {
            dev_err(dev, "Failed to set GPIO direction\n");
            goto err_free_fifos;
        }

        priv->irq = gpio_to_irq(priv->gpio_int);
        ret = devm_request_irq(dev, priv->irq, ch432_interrupt,
                              IRQ_TYPE_EDGE_FALLING | IRQF_ONESHOT,
                              DRIVER_NAME, priv);
        if (ret) {
            dev_err(dev, "Failed to request IRQ %d\n", priv->irq);
            goto err_free_fifos;
        }
    } else {
        priv->irq = -1;
        dev_info(dev, "No interrupt GPIO configured - using polling\n");
    }

    // Initialize hardware
    ch432_init_port(priv);

    // Register TTY device
    tty_port_link_device(&priv->port, ch432_tty_driver, index);
    ch432_ports[index] = priv;

    dev_info(dev, "CH432 SPI-to-RS485 converter probed successfully (port %d)\n", index);
    return 0;

err_free_fifos:
    kfifo_free(&priv->rx_fifo);
    kfifo_free(&priv->tx_fifo);
    tty_port_destroy(&priv->port);
    return ret;
}

static void ch432_remove(struct spi_device *spi)
{
    struct ch432_port *priv = spi_get_drvdata(spi);
    int i;

    for (i = 0; i < MAX_PORTS; i++) {
        if (ch432_ports[i] == priv) {
            ch432_ports[i] = NULL;
            break;
        }
    }

    tty_port_destroy(&priv->port);
    kfifo_free(&priv->rx_fifo);
    kfifo_free(&priv->tx_fifo);
}

static const struct spi_device_id ch432_spi_id[] = {
    { "ch432_spi", 0 },
    { }
};
MODULE_DEVICE_TABLE(spi, ch432_spi_id);

static const struct of_device_id ch432_dt_ids[] = {
    { .compatible = "ch432_spi", },
    { }
};
MODULE_DEVICE_TABLE(of, ch432_dt_ids);

static struct spi_driver ch432_spi_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = ch432_dt_ids,
    },
    .probe = ch432_probe,
    .remove = ch432_remove,
    .id_table = ch432_spi_id,
};

static int __init ch432_init(void)
{
    int ret;

    // Allocate TTY driver
    ch432_tty_driver = tty_alloc_driver(MAX_PORTS, TTY_DRIVER_REAL_RAW);
    if (IS_ERR(ch432_tty_driver))
        return PTR_ERR(ch432_tty_driver);

    ch432_tty_driver->driver_name = DRIVER_NAME;
    ch432_tty_driver->name = TTY_DEVICE_NAME;
    ch432_tty_driver->major = 0; // Auto-allocate
    ch432_tty_driver->minor_start = 0;
    ch432_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
    ch432_tty_driver->subtype = SERIAL_TYPE_NORMAL;
    ch432_tty_driver->init_termios = tty_std_termios;
    ch432_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
    tty_set_operations(ch432_tty_driver, &ch432_tty_ops);

    // Register TTY driver
    ret = tty_register_driver(ch432_tty_driver);
    if (ret) {
        tty_driver_kref_put(ch432_tty_driver);
        return ret;
    }

    // Register SPI driver
    ret = spi_register_driver(&ch432_spi_driver);
    if (ret) {
        tty_unregister_driver(ch432_tty_driver);
        tty_driver_kref_put(ch432_tty_driver);
        return ret;
    }

    return 0;
}

static void __exit ch432_exit(void)
{
    spi_unregister_driver(&ch432_spi_driver);
    tty_unregister_driver(ch432_tty_driver);
    tty_driver_kref_put(ch432_tty_driver);
}

module_init(ch432_init);
module_exit(ch432_exit);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("CH432 SPI to RS485/RS422 driver - Optimized for Modbus RTU");
MODULE_LICENSE("GPL");
