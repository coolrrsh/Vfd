#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <pthread.h>
#include <time.h>

// For GPIO on Raspberry Pi
// You can install and use either wiringPi or pigpio or a direct sysfs approach.
// We'll assume wiringPi for this example, but you can adapt if needed.
#include <wiringPi.h>
#include <wiringPiSPI.h>

/******************************************************************************************
 *  ALL REGISTER DEFINITIONS, BITFIELDS, STRUCTS, AND CONSTANTS FROM THE PYTHON DRIVER    *
 ******************************************************************************************/

// CH432T register definitions
#define CH432T_RBR_REG 0x00  // RX FIFO
#define CH432T_THR_REG 0x00  // TX FIFO
#define CH432T_IER_REG 0x01  // Interrupt Enable
#define CH432T_IIR_REG 0x02  // Interrupt Identification
#define CH432T_FCR_REG 0x02  // FIFO Control
#define CH432T_LCR_REG 0x03  // Line Control
#define CH432T_MCR_REG 0x04  // Modem Control
#define CH432T_LSR_REG 0x05  // Line Status
#define CH432T_MSR_REG 0x06  // Modem Status
#define CH432T_SCR_REG 0x07  // Scratch Pad
// Special Register set: Only if (LCR[7] == 1)
#define CH432T_DLL_REG 0x00  // Divisor Latch Low
#define CH432T_DLH_REG 0x01  // Divisor Latch High

// IER register bits
#define CH432T_IER_RDI_BIT   (1 << 0) // Enable RX data interrupt
#define CH432T_IER_THRI_BIT  (1 << 1) // Enable TX holding register interrupt
#define CH432T_IER_RLSI_BIT  (1 << 2) // Enable RX line status interrupt
#define CH432T_IER_MSI_BIT   (1 << 3) // Enable Modem status interrupt
// IER enhanced register bits
#define CH432T_IER_RESET_BIT     (1 << 7)
#define CH432T_IER_LOWPOWER_BIT  (1 << 6)
#define CH432T_IER_SLEEP_BIT     (1 << 5)
#define CH432T_IER_CK2X_BIT      (1 << 5)

#define CH432T_LOW_POWER_MODE    CH432T_IER_LOWPOWER_BIT  // low power mode
#define CH432T_SLEEP_MODE        CH432T_IER_SLEEP_BIT     // sleep mode
#define CH432T_STANDARD_MODE     0

// IIR register bits
#define CH432T_IIR_ID_MASK    0x0E
#define CH432T_IIR_NO_INT_BIT (1 << 0)
#define CH432T_IIR_RLSE_SRC   0x06 // RX line status error
#define CH432T_IIR_RDI_SRC    0x04 // RX data interrupt
#define CH432T_IIR_RTOI_SRC   0x0C // RX time-out interrupt
#define CH432T_IIR_THRI_SRC   0x02 // TX holding register empty
#define CH432T_IIR_MSI_SRC    0x00 // Modem status interrupt

// FCR register bits
#define CH432T_FCR_FIFO_BIT       (1 << 0) // Enable FIFO
#define CH432T_FCR_RXRESET_BIT    (1 << 1) // Reset RX FIFO
#define CH432T_FCR_TXRESET_BIT    (1 << 2) // Reset TX FIFO
#define CH432T_FCR_RXLVL_BIT      (0x03 << 6) // RX Trigger level
// Trigger point settings
#define CH432T_FCR_RECVTG_LEN_1   (0x00 << 6)
#define CH432T_FCR_RECVTG_LEN_4   (0x01 << 6)
#define CH432T_FCR_RECVTG_LEN_8   (0x02 << 6)
#define CH432T_FCR_RECVTG_LEN_14  (0x03 << 6)

// LCR register bits
#define CH432T_LCR_LENGTH_BIT       (0x03 << 0)
#define CH432T_LCR_STOPLEN_BIT      (1 << 2)
#define CH432T_LCR_PARITY_EN_BIT    (1 << 3)
#define CH432T_LCR_PARITY_MODE_BIT  (0x03 << 4)
#define CH432T_LCR_TXBREAK_BIT      (1 << 6)
#define CH432T_LCR_DLAB_BIT         (1 << 7)
#define CH432T_LCR_CONF_MODE_A      CH432T_LCR_DLAB_BIT

// Word length definitions
#define CH432T_LCR_WORD_LEN_5 0x00
#define CH432T_LCR_WORD_LEN_6 0x01
#define CH432T_LCR_WORD_LEN_7 0x02
#define CH432T_LCR_WORD_LEN_8 0x03

// STOP bits
#define CH432T_STOPBIT_1 0
#define CH432T_STOPBIT_2 CH432T_LCR_STOPLEN_BIT

// Parity modes
#define CH432T_CHECKBIT_ODD   (0x00 << 4)
#define CH432T_CHECKBIT_EVEN  (0x01 << 4)
#define CH432T_CHECKBIT_MARK  (0x02 << 4)
#define CH432T_CHECKBIT_SPACE (0x03 << 4)

// MCR register bits
#define CH432T_MCR_DTR_BIT  (1 << 0)
#define CH432T_MCR_RTS_BIT  (1 << 1)
#define CH432T_MCR_OUT1     (1 << 2)
#define CH432T_MCR_OUT2     (1 << 3)
#define CH432T_MCR_LOOP_BIT (1 << 4)
#define CH432T_MCR_AFE      (1 << 5)

// LSR register bits
#define CH432T_LSR_DR_BIT      (1 << 0)
#define CH432T_LSR_OE_BIT      (1 << 1)
#define CH432T_LSR_PE_BIT      (1 << 2)
#define CH432T_LSR_FE_BIT      (1 << 3)
#define CH432T_LSR_BI_BIT      (1 << 4)
#define CH432T_LSR_THRE_BIT    (1 << 5)
#define CH432T_LSR_TEMT_BIT    (1 << 6)
#define CH432T_LSR_FIFOE_BIT   (1 << 7)
#define CH432T_LSR_BRK_ERROR_MASK 0x1E

// MSR register bits
#define CH432T_MSR_DCTS_BIT  (1 << 0)
#define CH432T_MSR_DDSR_BIT  (1 << 1)
#define CH432T_MSR_DRI_BIT   (1 << 2)
#define CH432T_MSR_DCD_BIT   (1 << 3)
#define CH432T_MSR_CTS_BIT   (1 << 4)
#define CH432T_MSR_DSR_BIT   (1 << 5)
#define CH432T_MSR_RI_BIT    (1 << 6)
#define CH432T_MSR_CD_BIT    (1 << 7)
#define CH432T_MSR_DELTA_MASK 0x0F

// CH432T Ports
#define CH432T_PORT_1 0
#define CH432T_PORT_2 1
// External input clock frequency or external crystal frequency
#define CH432T_CLOCK_FREQUENCY 22118400

// Misc definitions
#define CH432T_FIFO_SIZE   16
#define CH432T_REG_SHIFT   2

// Supported parity types, for convenience
typedef enum {
    PARITY_NONE = 0,
    PARITY_ODD,
    PARITY_EVEN,
    PARITY_MARK,
    PARITY_SPACE
} CH432T_Parity;

// Stop bits
typedef enum {
    STOPBITS_ONE = 1,
    STOPBITS_ONE_POINT_FIVE,
    STOPBITS_TWO
} CH432T_StopBits;

// Data bits
typedef enum {
    DATABITS_5 = 5,
    DATABITS_6,
    DATABITS_7,
    DATABITS_8
} CH432T_DataBits;

// For convenience, we'll replicate the struct-based register approach as normal C usage.
// We'll define them as bitfields, but we won't use them to do real-time reading/writing from the device.
// Instead, we'll define standard read/write functions with the same logic.

// We'll define a struct that holds all relevant configuration and state.

// We'll define a lock for the SPI
static pthread_mutex_t ch432t_spi_lock = PTHREAD_MUTEX_INITIALIZER;

// We'll define the SPI device path and speed
static const char *SPI_DEVICE = "/dev/spidev0.0";
static uint32_t SPI_SPEED = 50000; // 1 MHz default

// We'll define a struct that represents the CH432T driver instance.
// The Python code sets up a class with methods. We'll do something similar in C.

typedef struct {
    int spi_fd;             // SPI file descriptor
    int cs_pin;             // Chip-select pin (BCM)
    int portnum;            // CH432T_PORT_1 or CH432T_PORT_2 (0 or 1)

    // user config
    unsigned int baudrate;
    CH432T_DataBits bytesize;
    CH432T_Parity parity;
    CH432T_StopBits stopbits;
    bool is_open;
} DFRobot_CH432T;

/********************* SPI read/write helper functions *********************/

static int spi_open_device(const char *device, uint32_t speed) {
    int fd = open(device, O_RDWR);
    if (fd < 0) {
        perror("Failed to open SPI device");
        return -1;
    }
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) {
        perror("Failed to set SPI speed");
        close(fd);
        return -1;
    }
    uint8_t mode = 0; // mode 0
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1) {
        perror("Failed to set SPI mode");
        close(fd);
        return -1;
    }
    uint8_t bits = 8; // 8 bits per word
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) {
        perror("Failed to set bits per word");
        close(fd);
        return -1;
    }
    return fd;
}

static void spi_close_device(int fd) {
    if (fd >= 0) {
        close(fd);
    }
}

static void spi_transfer(int fd, uint8_t *tx, uint8_t *rx, size_t length, uint32_t speed_hz) {
    struct spi_ioc_transfer transfer;
    memset(&transfer, 0, sizeof(transfer));

    transfer.tx_buf = (unsigned long)tx;
    transfer.rx_buf = (unsigned long)rx;
    transfer.len = length;
    transfer.speed_hz = speed_hz;
    transfer.delay_usecs = 0;
    transfer.bits_per_word = 8;

    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
    if (ret < 0) {
        perror("SPI transfer failed");
    }
}

/*
static uint8_t read_register(DFRobot_CH432T *handle, uint8_t reg) {
    pthread_mutex_lock(&ch432t_spi_lock);

    // Calculation for register address: ( (reg + portnum*0x08) << CH432T_REG_SHIFT )
    // Then 0xFD & that for read.
    uint8_t real_reg = (uint8_t)(( (reg + (handle->portnum * 0x08)) << CH432T_REG_SHIFT ) & 0xFD);

    uint8_t tx[2];
    uint8_t rx[2];
    tx[0] = real_reg;
    tx[1] = 0xFF;
    rx[0] = 0;
    rx[1] = 0;

    // Chip-select
    digitalWrite(handle->cs_pin, LOW);
    spi_transfer(handle->spi_fd, tx, rx, 2, SPI_SPEED);
    digitalWrite(handle->cs_pin, HIGH);

    uint8_t read_val = rx[1];

    pthread_mutex_unlock(&ch432t_spi_lock);
    return read_val;
}
*/

static uint8_t read_register(DFRobot_CH432T *handle, uint8_t reg) {
    pthread_mutex_lock(&ch432t_spi_lock);

    uint8_t buf[2];
    buf[0] = 0xFD & ((reg + handle->portnum * 0x08) << CH432T_REG_SHIFT);
    buf[1] = 0xFF;

    digitalWrite(handle->cs_pin, LOW);
    wiringPiSPIDataRW(handle->portnum, buf, 2);
    digitalWrite(handle->cs_pin, HIGH);

    pthread_mutex_unlock(&ch432t_spi_lock);
    return buf[1];
}


static void read_register_buffer(DFRobot_CH432T *handle, uint8_t reg, uint8_t *rx_buf, size_t len) {
    // Variation to read multiple bytes from the same register?
    // The python code only does single byte or repeated single bytes.
    // We'll do a repeated approach.

    for (size_t i = 0; i < len; i++) {
        rx_buf[i] = read_register(handle, reg);
    }
}

/*
static void write_register_single(DFRobot_CH432T *handle, uint8_t reg, uint8_t data) {
    pthread_mutex_lock(&ch432t_spi_lock);

    uint8_t real_reg = (uint8_t)(0x02 | ( ((reg + (handle->portnum * 0x08)) << CH432T_REG_SHIFT) ));

    uint8_t tx[2];
    uint8_t rx[2];
    tx[0] = real_reg;
    tx[1] = data;
    rx[0] = 0;
    rx[1] = 0;

    digitalWrite(handle->cs_pin, LOW);
    spi_transfer(handle->spi_fd, tx, rx, 2, SPI_SPEED);
    digitalWrite(handle->cs_pin, HIGH);

    pthread_mutex_unlock(&ch432t_spi_lock);
    usleep(1000); // 1 ms
}
*/

static void write_register_single(DFRobot_CH432T *handle, uint8_t reg, uint8_t data) {
    pthread_mutex_lock(&ch432t_spi_lock);

    uint8_t buf[2];
    buf[0] = 0x02 | ((reg + handle->portnum * 0x08) << CH432T_REG_SHIFT);
    buf[1] = data;

    digitalWrite(handle->cs_pin, LOW);
    wiringPiSPIDataRW(handle->portnum, buf, 2);
    digitalWrite(handle->cs_pin, HIGH);

    pthread_mutex_unlock(&ch432t_spi_lock);
    delay(1); // 1 ms
}


static void write_register_buffer(DFRobot_CH432T *handle, uint8_t reg, const uint8_t *data, size_t len) {
    // The python driver inserts the register address at the front, then data.
    // We'll replicate that approach, but each write is done one at a time to replicate the python logic.

    for (size_t i = 0; i < len; i++) {
        write_register_single(handle, reg, data[i]);
    }
}

// Helper to update specific bits in a register
static void reg_bit_update(DFRobot_CH432T *handle, uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t temp = read_register(handle, reg);
    temp &= (uint8_t)(~mask);
    temp |= (uint8_t)(value & mask);
    write_register_single(handle, reg, temp);
}

/********************* CH432T driver core functions *********************/

// create & init driver instance
DFRobot_CH432T *CH432T_create(int portnum, unsigned int baudrate, CH432T_DataBits bytesize,
                              CH432T_Parity parity, CH432T_StopBits stopbits) {
    // You can adjust the CS pin if needed.
    DFRobot_CH432T *handle = (DFRobot_CH432T*)calloc(1, sizeof(DFRobot_CH432T));
    handle->spi_fd = -1;
    handle->cs_pin = 8; // default
    handle->portnum = portnum; // typically 0 or 1 in python code
    handle->baudrate = baudrate;
    handle->bytesize = bytesize;
    handle->parity = parity;
    handle->stopbits = stopbits;
    handle->is_open = false;

    
    return handle;
}


void CH432T_destroy(DFRobot_CH432T *handle) {
    if (!handle) return;
    free(handle);
}

// internal function that sets up the device once the SPI is open.
static void ch432t_reconfigure_port(DFRobot_CH432T *handle);


// open device
bool CH432T_open(DFRobot_CH432T *handle) {
    if (!handle) return false;
    if (handle->is_open) {
        // already open
        return true;
    }

    // initialize wiringPi
    wiringPiSetupGpio(); 
    wiringPiSPISetup(handle->portnum, SPI_SPEED);
    
    pinMode(handle->cs_pin, OUTPUT);
    digitalWrite(handle->cs_pin, HIGH);

    // open SPI
    handle->spi_fd = spi_open_device(SPI_DEVICE, SPI_SPEED);
    if (handle->spi_fd < 0) {
        fprintf(stderr, "Failed to open SPI device %s\n", SPI_DEVICE);
        return false;
    }

    // check read of some registers to ensure device is responding
    // read the IIR and LSR
    uint8_t iir = read_register(handle, CH432T_IIR_REG);
    uint8_t lsr = read_register(handle, CH432T_LSR_REG);
    // test user register (SCR)
    write_register_single(handle, CH432T_SCR_REG, 0x66);
    uint8_t scr = read_register(handle, CH432T_SCR_REG);
    if (scr != 0x66) {
	printf("scr reads 0x%x\n",scr);
        fprintf(stderr, "Failed to open CH432T port: check connections, etc.\n");
        // close
        spi_close_device(handle->spi_fd);
        handle->spi_fd = -1;
        return false;
    }

    // now configure port
    ch432t_reconfigure_port(handle);
    handle->is_open = true;
    return true;
}

// close device
void CH432T_close(DFRobot_CH432T *handle) {
    if (!handle || !handle->is_open) return;
    // set low power mode before close.
    reg_bit_update(handle, CH432T_IER_REG, CH432T_IER_LOWPOWER_BIT, CH432T_LOW_POWER_MODE);

    // close SPI fd
    spi_close_device(handle->spi_fd);
    handle->spi_fd = -1;
    handle->is_open = false;
}

// reconfigure port (similar to python _reconfigure_port)
static void ch432t_reconfigure_port(DFRobot_CH432T *handle) {
    // compute cflag from bytesize, stopbits, parity

    // word length
    uint8_t cflag = 0;
    switch (handle->bytesize) {
        case DATABITS_5: cflag |= CH432T_LCR_WORD_LEN_5; break;
        case DATABITS_6: cflag |= CH432T_LCR_WORD_LEN_6; break;
        case DATABITS_7: cflag |= CH432T_LCR_WORD_LEN_7; break;
        case DATABITS_8: cflag |= CH432T_LCR_WORD_LEN_8; break;
        default: cflag |= CH432T_LCR_WORD_LEN_8; break; // fallback
    }

    // stop bits
    if (handle->stopbits == STOPBITS_ONE) {
        cflag |= CH432T_STOPBIT_1;
    } else {
        // for both 1.5 and 2, we do 2 stop bits
        cflag |= CH432T_STOPBIT_2;
    }

    // parity
    switch (handle->parity) {
        case PARITY_NONE:
            // do nothing (no parity)
            break;
        case PARITY_ODD:
            cflag |= (CH432T_LCR_PARITY_EN_BIT | CH432T_CHECKBIT_ODD);
            break;
        case PARITY_EVEN:
            cflag |= (CH432T_LCR_PARITY_EN_BIT | CH432T_CHECKBIT_EVEN);
            break;
        case PARITY_MARK:
            cflag |= (CH432T_LCR_PARITY_EN_BIT | CH432T_CHECKBIT_MARK);
            break;
        case PARITY_SPACE:
            cflag |= (CH432T_LCR_PARITY_EN_BIT | CH432T_CHECKBIT_SPACE);
            break;
    }

    // write LCR
    write_register_single(handle, CH432T_LCR_REG, cflag);

    // reset FIFOs, enable FIFOs, set trigger to 8
    uint8_t fcr_val = (CH432T_FCR_FIFO_BIT | CH432T_FCR_RXRESET_BIT | CH432T_FCR_TXRESET_BIT | CH432T_FCR_RECVTG_LEN_8);
    write_register_single(handle, CH432T_FCR_REG, fcr_val);

    // enable certain interrupts
    // (the python code used RDI_BIT, RLSI_BIT, MSI_BIT. not THRI_BIT by default)
    uint8_t ier_val = (CH432T_IER_RDI_BIT | CH432T_IER_RLSI_BIT | CH432T_IER_MSI_BIT);
    write_register_single(handle, CH432T_IER_REG, ier_val);

    // enable RTS, OUT2, AFE
    uint8_t mcr_val = (CH432T_MCR_RTS_BIT | CH432T_MCR_OUT2 | CH432T_MCR_AFE);
    write_register_single(handle, CH432T_MCR_REG, mcr_val);

    // set baud rate
    // replicate set_baudrate function below
    // check if we need prescaler

    double base_clock = (double)CH432T_CLOCK_FREQUENCY / 12.0; // CK2X=0
    bool ck2x = false;
    if (handle->baudrate > 115200) {
        // CK2X=1
        ck2x = true;
        base_clock = (double)CH432T_CLOCK_FREQUENCY / 12.0 * 24.0; // effectively 22118400
    }
    // update CK2X if needed
    // if port 1 => reg = CH432T_IER_REG + 0x08
    // else reg = CH432T_IER_REG
    uint8_t reg_offset = (handle->portnum == 1) ? (CH432T_IER_REG + 0x08) : CH432T_IER_REG;
    if (ck2x) {
        reg_bit_update(handle, reg_offset, CH432T_IER_CK2X_BIT, CH432T_IER_CK2X_BIT);
    } else {
        reg_bit_update(handle, reg_offset, CH432T_IER_CK2X_BIT, 0);
    }

    // set the LCR to open DLAB
    uint8_t old_lcr = read_register(handle, CH432T_LCR_REG);
    write_register_single(handle, CH432T_LCR_REG, CH432T_LCR_CONF_MODE_A);
    usleep(2000);

    // compute divisor
    // base_clock / 16 / baud
    double divisor = (base_clock / 16.0) / (double)handle->baudrate;
    uint16_t mode = (uint16_t)(divisor + 0.5);
    write_register_single(handle, CH432T_DLL_REG, (uint8_t)(mode & 0xFF));
    write_register_single(handle, CH432T_DLH_REG, (uint8_t)((mode >> 8) & 0xFF));

    // restore LCR
    write_register_single(handle, CH432T_LCR_REG, old_lcr);
}

// convenience function to set baudrate after open
void CH432T_set_baudrate(DFRobot_CH432T *handle, unsigned int baud) {
    if (!handle || !handle->is_open) return;
    handle->baudrate = baud;
    ch432t_reconfigure_port(handle);
}

// flush TX buffer
void CH432T_reset_output_buffer(DFRobot_CH432T *handle) {
    if (!handle || !handle->is_open) return;
    reg_bit_update(handle, CH432T_FCR_REG, CH432T_FCR_TXRESET_BIT, CH432T_FCR_TXRESET_BIT);
}

// flush RX buffer
void CH432T_reset_input_buffer(DFRobot_CH432T *handle) {
    if (!handle || !handle->is_open) return;
    reg_bit_update(handle, CH432T_FCR_REG, CH432T_FCR_RXRESET_BIT, CH432T_FCR_RXRESET_BIT);
}

// read from device (blocking) in a simplified manner.
// The python code uses a complicated approach with timeouts.
// We'll do a simpler approach.
// Returns number of bytes read, or -1 on error.
int CH432T_read(DFRobot_CH432T *handle, uint8_t *buf, size_t size, unsigned int timeout_ms) {
    if (!handle || !handle->is_open) return -1;
    if (size == 0) return 0;

    // We'll do a polling approach. The python code uses an interrupt-based approach.
    // We'll read the LSR data_ready bit in a loop until we get the requested number of bytes or we time out.

    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    size_t read_count = 0;

    while (read_count < size) {
        // check LSR
        uint8_t lsr = read_register(handle, CH432T_LSR_REG);
        // if data_ready bit is set, read from RBR
        if (lsr & CH432T_LSR_DR_BIT) {
		printf("data_ready bit is set in CH432T_LSR_DR_BIT\n");
            buf[read_count] = read_register(handle, CH432T_RBR_REG);
            read_count++;
        } else {
            // check for time out
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            unsigned long elapsed_ms = (unsigned long)((now.tv_sec - start.tv_sec) * 1000UL)
                                      + (unsigned long)((now.tv_nsec - start.tv_nsec) / 1000000UL);
            if (elapsed_ms >= timeout_ms) {
                break; // time out
            }
            usleep(1000);
        }
    }

    return (int)read_count;
}

// write to device
// returns number of bytes actually written.
int CH432T_write(DFRobot_CH432T *handle, const uint8_t *data, size_t size) {
    if (!handle || !handle->is_open) return -1;

    // The python code writes data by writing to the THR register for each byte.
    // We'll do the same here.
    for (size_t i = 0; i < size; i++) {
        write_register_single(handle, CH432T_THR_REG, data[i]);
    }

    return (int)size;
}

/*********************************************************************
 *  Additional utility functions from the python driver (optional)   *
 *  set_low_power_mode, set_sleep_mode, enable interrupts, etc.      *
 *********************************************************************/

void CH432T_set_low_power_mode(DFRobot_CH432T *handle, uint8_t mode) {
    if (!handle || !handle->is_open) return;
    reg_bit_update(handle, CH432T_IER_REG, CH432T_IER_LOWPOWER_BIT, mode);
}

// ... replicate additional advanced methods if needed ...

/*********************************************************************
 *  Example usage / Testing code in main()                           *
 *********************************************************************/
#define SLAVE_ID 100
#define FUNC_WRITE_SINGLE_REGISTER 0x06
#define REGISTER_LOGIC_COMMAND 8192

// Function to calculate CRC16 for Modbus RTU
uint16_t calculate_crc16(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

int main(){

     DFRobot_CH432T *ch432 = CH432T_create(CH432T_PORT_1, 9600, DATABITS_8, PARITY_NONE, STOPBITS_ONE);
    if (!CH432T_open(ch432)) {
        printf("Failed to open CH432T device.\n");
        CH432T_destroy(ch432);
        return -1;
    }
    printf("CH432T port opened successfully.\n");

    // Prepare Modbus RTU packet
    uint8_t modbus_packet[8];
    uint8_t read_buf[100];
    uint16_t crc;
    int len;
    modbus_packet[0] = 100;
    modbus_packet[1] = 0x06;
    modbus_packet[2] = 0x20; //(REGISTER_LOGIC_COMMAND >> 8) & 0xFF;
    modbus_packet[3] = 0x00; //REGISTER_LOGIC_COMMAND & 0xFF;
    modbus_packet[4] = 0x00;
    modbus_packet[5] = 0x06;  // Example: Start Command

    crc = calculate_crc16(modbus_packet, 6);
    modbus_packet[6] = crc & 0xFF;
    modbus_packet[7] = (crc >> 8) & 0xFF;

    // Send Modbus packet over CH432T
    CH432T_write(ch432, modbus_packet, sizeof(modbus_packet));
    printf("Modbus command sent to VFD.\n");

    // Read response (non-blocking for ~1 second)
    memset(read_buf, 0, sizeof(read_buf));
    len = CH432T_read(ch432, read_buf, 10, 1000);
    if (len > 0) {
        printf("Read %d bytes: ", len);
        for (int i = 0; i < len; i++) {
            printf("%02X ", read_buf[i]);
        }
        printf("\n");
    } else {
        printf("No data or read timed out.\n");
    }

    
    modbus_packet[0] = 100;
    modbus_packet[1] = 0x06;
    modbus_packet[2] = 0x20; //(REGISTER_LOGIC_COMMAND >> 8) & 0xFF;
    modbus_packet[3] = 0x00; //REGISTER_LOGIC_COMMAND & 0xFF;
    modbus_packet[4] = 0x00;
    modbus_packet[5] = 0x05;  // Example: Start Command

    crc = calculate_crc16(modbus_packet, 6);
    modbus_packet[6] = crc & 0xFF;
    modbus_packet[7] = (crc >> 8) & 0xFF;

    // Send Modbus packet over CH432T
    CH432T_write(ch432, modbus_packet, sizeof(modbus_packet));
    printf("Modbus command sent to VFD.\n");

    // Read response (non-blocking for ~1 second)
    memset(read_buf, 0, sizeof(read_buf));
    len = CH432T_read(ch432, read_buf, 10, 1000);
    if (len > 0) {
        printf("Read %d bytes: ", len);
        for (int i = 0; i < len; i++) {
            printf("%02X ", read_buf[i]);
        }
        printf("\n");
    } else {
        printf("No data or read timed out.\n");
    }


    CH432T_destroy(ch432);
    return 0;
}
