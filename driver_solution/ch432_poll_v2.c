// SPDX-License-Identifier: GPL-2.0
/*
 * CH432 SPI to RS485/RS422 driver - Optimized for Modbus RTU
 * With proper SPI timing and RS485 direction control
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/kfifo.h>

#define DRIVER_NAME "ch432_spi"
#define TTY_DEVICE_NAME "ttyWCH"
#define MAX_PORTS 2
#define SPI_OPERATION_DELAY_US 10

// CH432 Register definitions
#define CH432_RBR 0x00
#define CH432_THR 0x00
#define CH432_IER 0x01
#define CH432_IIR 0x02
#define CH432_FCR 0x02
#define CH432_LCR 0x03
#define CH432_MCR 0x04
#define CH432_LSR 0x05
#define CH432_MSR 0x06
#define CH432_SCR 0x07
#define CH432_DLL 0x00
#define CH432_DLH 0x01

// Register bit definitions
#define CH432_LSR_DR      BIT(0)  // Data ready
#define CH432_LSR_THRE    BIT(5)  // Transmitter holding register empty
#define CH432_LSR_TEMT    BIT(6)  // Transmitter empty
#define CH432_FCR_FIFO_EN BIT(0)  // FIFO enable
#define CH432_FCR_RXCLR   BIT(1)  // Clear receive FIFO
#define CH432_FCR_TXCLR   BIT(2)  // Clear transmit FIFO
#define CH432_MCR_RTS     BIT(1)  // Request to send
#define CH432_MCR_AFE     BIT(5)  // Auto flow control enable
#define CH432_LCR_DLAB    BIT(7)  // Divisor latch access bit

#define CH432_CLOCK_FREQ  18432000

struct ch432_port {
    struct spi_device *spi;
    struct tty_port port;
    struct kfifo rx_fifo;
    struct mutex lock;
    int gpio_de;
    int gpio_re;
    u8 portnum;
    bool tx_active;
};

static struct tty_driver *ch432_tty_driver;
static struct ch432_port *ch432_ports[MAX_PORTS];
static void ch432_write_reg(struct ch432_port *priv, u8 reg, u8 val);

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

// Helper functions for TTY operations
static unsigned int ch432_tty_write_room(struct tty_struct *tty)
{
    return 4096;
}

static unsigned int ch432_tty_chars_in_buffer(struct tty_struct *tty)
{
    return 0;
}

static void ch432_tty_set_termios(struct tty_struct *tty, const struct ktermios *old_termios)
{
    struct ch432_port *priv = tty->driver_data;
    u8 lcr = 0;
    u32 baud;

    if (!priv)
        return;

    // Set data bits
    switch (tty->termios.c_cflag & CSIZE) {
    case CS5: lcr |= 0x00; break;
    case CS6: lcr |= 0x01; break;
    case CS7: lcr |= 0x02; break;
    case CS8: lcr |= 0x03; break;
    }

    // Set stop bits
    if (tty->termios.c_cflag & CSTOPB)
        lcr |= 0x04;

    // Set parity
    if (tty->termios.c_cflag & PARENB) {
        lcr |= 0x08;
        if (!(tty->termios.c_cflag & PARODD))
            lcr |= 0x10; // Even parity
    }

    // Set new LCR
    ch432_write_reg(priv, CH432_LCR, lcr);

    // Set baud rate
    baud = tty_get_baud_rate(tty);
    if (baud == 0)
        baud = 9600;

    u32 divisor = CH432_CLOCK_FREQ / 16 / baud;
    
    // Access divisor latch
    ch432_write_reg(priv, CH432_LCR, lcr | CH432_LCR_DLAB);
    
    // Set divisor
    ch432_write_reg(priv, CH432_DLL, divisor & 0xFF);
    ch432_write_reg(priv, CH432_DLH, (divisor >> 8) & 0xFF);
    
    // Restore LCR
    ch432_write_reg(priv, CH432_LCR, lcr);
}

// SPI register access with proper delays and error checking
static int ch432_spi_transfer(struct ch432_port *priv, u8 *tx_buf, u8 *rx_buf, u8 len)
{
    struct spi_transfer xfer = {
        .tx_buf = tx_buf,
        .rx_buf = rx_buf,
        .len = len,
        .delay = {
            .value = SPI_OPERATION_DELAY_US,
            .unit = SPI_DELAY_UNIT_USECS,
        },
    };
    struct spi_message msg;

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    return spi_sync(priv->spi, &msg);
}

static u8 ch432_read_reg(struct ch432_port *priv, u8 reg)
{
    u8 tx_buf[2] = {0};
    u8 rx_buf[2] = {0};
    int ret;

    // Format read command
    tx_buf[0] = 0x80 | (reg << 2) | (priv->portnum ? 0x40 : 0x00);
    
    ret = ch432_spi_transfer(priv, tx_buf, rx_buf, 2);
    if (ret) {
        dev_err(&priv->spi->dev, "SPI read error: %d\n", ret);
        return 0xFF;
    }

    udelay(SPI_OPERATION_DELAY_US);
    return rx_buf[1];
}

static void ch432_write_reg(struct ch432_port *priv, u8 reg, u8 val)
{
    u8 tx_buf[2] = {0};
    int ret;

    // Format write command
    tx_buf[0] = (reg << 2) | (priv->portnum ? 0x40 : 0x00);
    tx_buf[1] = val;

    ret = ch432_spi_transfer(priv, tx_buf, NULL, 2);
    if (ret) {
        dev_err(&priv->spi->dev, "SPI write error: %d\n", ret);
    }
    udelay(SPI_OPERATION_DELAY_US);
}

// RS485 direction control
static void ch432_rs485_enable_tx(struct ch432_port *priv)
{
    if (gpio_is_valid(priv->gpio_de))
        gpio_set_value(priv->gpio_de, 1);
    if (gpio_is_valid(priv->gpio_re))
        gpio_set_value(priv->gpio_re, 1);
    udelay(50); // Critical delay for transceiver enable
}

static void ch432_rs485_enable_rx(struct ch432_port *priv)
{
    if (gpio_is_valid(priv->gpio_de))
        gpio_set_value(priv->gpio_de, 0);
    if (gpio_is_valid(priv->gpio_re))
        gpio_set_value(priv->gpio_re, 0);
    udelay(50); // Critical delay before receiving
}

// Hardware initialization with verification
static int ch432_init_port(struct ch432_port *priv)
{
    int retry = 3;
    u8 scr;

    // Test communication with scratch register
    while (retry--) {
        ch432_write_reg(priv, CH432_SCR, 0x5A);
        udelay(1000);
        scr = ch432_read_reg(priv, CH432_SCR);
        
        if (scr == 0x5A) break;
        
        dev_warn(&priv->spi->dev, "SCR test failed (attempt %d): wrote 0x5A, read 0x%02X\n", 
                3 - retry, scr);
        msleep(10);
    }

    if (scr != 0x5A) {
        dev_err(&priv->spi->dev, "CH432 communication failed!\n");
        return -EIO;
    }

    // Reset FIFOs
    ch432_write_reg(priv, CH432_FCR, CH432_FCR_FIFO_EN | CH432_FCR_RXCLR | CH432_FCR_TXCLR);
    udelay(1000);

    // Configure 9600 8N1 (Modbus default)
    ch432_write_reg(priv, CH432_LCR, 0x03); // 8N1
    
    // Set baud rate
    u32 divisor = CH432_CLOCK_FREQ / 16 / 9600;
    ch432_write_reg(priv, CH432_LCR, 0x03 | CH432_LCR_DLAB);
    ch432_write_reg(priv, CH432_DLL, divisor & 0xFF);
    ch432_write_reg(priv, CH432_DLH, (divisor >> 8) & 0xFF);
    ch432_write_reg(priv, CH432_LCR, 0x03);

    // Enable RS485 mode
    ch432_write_reg(priv, CH432_MCR, CH432_MCR_RTS | CH432_MCR_AFE);

    // Disable interrupts (polling mode)
    ch432_write_reg(priv, CH432_IER, 0x00);

    return 0;
}

// TTY operations
static int ch432_tty_open(struct tty_struct *tty, struct file *filp)
{
    struct ch432_port *priv = ch432_ports[tty->index];
    return priv ? tty_port_open(&priv->port, tty, filp) : -ENODEV;
}

static void ch432_tty_close(struct tty_struct *tty, struct file *filp)
{
    struct ch432_port *priv = tty->driver_data;
    if (priv) tty_port_close(&priv->port, tty, filp);
}

static ssize_t ch432_tty_write(struct tty_struct *tty, const u8 *buf, size_t count)
{
    struct ch432_port *priv = tty->driver_data;
    size_t i;
    int timeout;
    u8 mcr;

    if (!priv || !count)
        return -ENODEV;

    mutex_lock(&priv->lock);
    priv->tx_active = true;
    
    // Enable transmitter
    ch432_rs485_enable_tx(priv);
    mcr = ch432_read_reg(priv, CH432_MCR);
    ch432_write_reg(priv, CH432_MCR, mcr | CH432_MCR_RTS | CH432_MCR_AFE);
    udelay(50);

    // Transmit each byte with proper timing
    for (i = 0; i < count; i++) {
        timeout = 10000; // 10ms timeout
        
        while (!(ch432_read_reg(priv, CH432_LSR) & CH432_LSR_THRE)) {
            if (--timeout <= 0) {
                ch432_write_reg(priv, CH432_MCR, mcr);
                ch432_rs485_enable_rx(priv);
                priv->tx_active = false;
                mutex_unlock(&priv->lock);
                return -ETIMEDOUT;
            }
            udelay(10);
        }
        
        ch432_write_reg(priv, CH432_THR, buf[i]);
        udelay(100); // Inter-character delay (1ms at 9600 baud)
    }

    // Wait for complete transmission
    timeout = 10000;
    while (!(ch432_read_reg(priv, CH432_LSR) & CH432_LSR_TEMT)) {
        if (--timeout <= 0) break;
        udelay(10);
    }

    // Disable transmitter
    ch432_write_reg(priv, CH432_MCR, mcr);
    udelay(50); // Ensure last bit is transmitted
    ch432_rs485_enable_rx(priv);
    priv->tx_active = false;
    
    mutex_unlock(&priv->lock);
    return count;
}

static const struct tty_operations ch432_tty_ops = {
    .open = ch432_tty_open,
    .close = ch432_tty_close,
    .write = ch432_tty_write,
    .write_room = ch432_tty_write_room,
    .chars_in_buffer = ch432_tty_chars_in_buffer,
    .set_termios = ch432_tty_set_termios,
};

static int ch432_probe(struct spi_device *spi)
{
    struct device *dev = &spi->dev;
    struct ch432_port *priv;
    int ret, index;
    u32 val;

    // SPI configuration
    spi->mode = SPI_MODE_0;
    spi->max_speed_hz = 50000;
    if ((ret = spi_setup(spi)) < 0) {
        dev_err(dev, "SPI setup failed: %d\n", ret);
        return ret;
    }

    // Get port index
    if (of_property_read_u32(dev->of_node, "port-number", &val))
        index = 0;
    else if (val >= MAX_PORTS) {
        dev_err(dev, "Invalid port number %d\n", val);
        return -EINVAL;
    } else {
        index = val;
    }

    priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;

    spi_set_drvdata(spi, priv);
    priv->spi = spi;
    priv->portnum = index;
    mutex_init(&priv->lock);

    // Get RS485 direction control GPIOs
    priv->gpio_de = of_get_named_gpio(dev->of_node, "de-gpios", 0);
    priv->gpio_re = of_get_named_gpio(dev->of_node, "re-gpios", 0);

    if (gpio_is_valid(priv->gpio_de)) {
        if ((ret = devm_gpio_request(dev, priv->gpio_de, "ch432_de")))
            return ret;
        gpio_direction_output(priv->gpio_de, 0);
    }

    if (gpio_is_valid(priv->gpio_re)) {
        if ((ret = devm_gpio_request(dev, priv->gpio_re, "ch432_re")))
            return ret;
        gpio_direction_output(priv->gpio_re, 0);
    }

    // Initialize FIFO
    if ((ret = kfifo_alloc(&priv->rx_fifo, 4096, GFP_KERNEL)))
        return ret;

    // Initialize TTY port
    tty_port_init(&priv->port);
    priv->port.ops = &ch432_port_ops;

    // Initialize hardware
    if ((ret = ch432_init_port(priv)))
        goto err_cleanup;

    // Register TTY device
    tty_port_link_device(&priv->port, ch432_tty_driver, index);
    ch432_ports[index] = priv;

    dev_info(dev, "CH432 port %d initialized\n", index);
    return 0;

err_cleanup:
    kfifo_free(&priv->rx_fifo);
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

    kfifo_free(&priv->rx_fifo);
    tty_port_destroy(&priv->port);
}

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
};

static int __init ch432_init(void)
{
    int ret;

    ch432_tty_driver = tty_alloc_driver(MAX_PORTS, TTY_DRIVER_REAL_RAW);
    if (IS_ERR(ch432_tty_driver))
        return PTR_ERR(ch432_tty_driver);

    ch432_tty_driver->driver_name = DRIVER_NAME;
    ch432_tty_driver->name = TTY_DEVICE_NAME;
    ch432_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
    ch432_tty_driver->init_termios = tty_std_termios;
    ch432_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
    tty_set_operations(ch432_tty_driver, &ch432_tty_ops);

    if ((ret = tty_register_driver(ch432_tty_driver))) {
        tty_driver_kref_put(ch432_tty_driver);
        return ret;
    }

    if ((ret = spi_register_driver(&ch432_spi_driver))) {
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
MODULE_DESCRIPTION("CH432 SPI to RS485/RS422 driver");
MODULE_LICENSE("GPL");
