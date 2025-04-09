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

// VFD Registers
#define SLAVE_ID 100
#define FUNC_WRITE_SINGLE_REGISTER 0x06
#define REGISTER_LOGIC_COMMAND 8192

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

/*********************************************************************
 *                  Interrupt-Based Implementation                   *
 *********************************************************************/

// Define a lock for the SPI
static pthread_mutex_t ch432t_spi_lock = PTHREAD_MUTEX_INITIALIZER;

// Define the SPI device path and speed
static const char *SPI_DEVICE = "/dev/spidev0.0";
static uint32_t SPI_SPEED = 50000; // 1 MHz default

// Circular buffer structure for interrupt-based reading
typedef struct {
    uint8_t *buffer;
    size_t head;
    size_t tail;
    size_t size;
    bool full;
    pthread_mutex_t lock;
} CircularBuffer;

// CH432T driver instance with interrupt support
typedef struct {
    int spi_fd;             // SPI file descriptor
    int cs_pin;             // Chip-select pin (BCM)
    int portnum;            // CH432T_PORT_1 or CH432T_PORT_2 (0 or 1)
    int interrupt_pin;      // GPIO pin for interrupt (e.g., CTS pin)

    // user config
    unsigned int baudrate;
    CH432T_DataBits bytesize;
    CH432T_Parity parity;
    CH432T_StopBits stopbits;
    bool is_open;

    // interrupt handling
    CircularBuffer rx_buffer;
    volatile bool data_available;
    pthread_cond_t data_cond;
} DFRobot_CH432T;

/********************* Circular Buffer Functions *********************/

static void circular_buffer_init(CircularBuffer *cb, size_t size) {
    cb->buffer = (uint8_t *)malloc(size);
    cb->size = size;
    cb->head = 0;
    cb->tail = 0;
    cb->full = false;
    pthread_mutex_init(&cb->lock, NULL);
}

static void circular_buffer_free(CircularBuffer *cb) {
    if (cb->buffer) {
        free(cb->buffer);
    }
    pthread_mutex_destroy(&cb->lock);
}

static bool circular_buffer_write(CircularBuffer *cb, uint8_t data) {
    pthread_mutex_lock(&cb->lock);
    
    if (cb->full) {
        pthread_mutex_unlock(&cb->lock);
        return false;
    }
    
    cb->buffer[cb->head] = data;
    cb->head = (cb->head + 1) % cb->size;
    cb->full = (cb->head == cb->tail);
    
    pthread_mutex_unlock(&cb->lock);
    return true;
}

static bool circular_buffer_read(CircularBuffer *cb, uint8_t *data) {
    pthread_mutex_lock(&cb->lock);
    
    if (cb->head == cb->tail && !cb->full) {
        pthread_mutex_unlock(&cb->lock);
        return false;
    }
    
    *data = cb->buffer[cb->tail];
    cb->tail = (cb->tail + 1) % cb->size;
    cb->full = false;
    
    pthread_mutex_unlock(&cb->lock);
    return true;
}

static size_t circular_buffer_available(CircularBuffer *cb) {
    pthread_mutex_lock(&cb->lock);
    
    size_t count;
    if (cb->full) {
        count = cb->size;
    } else if (cb->head >= cb->tail) {
        count = cb->head - cb->tail;
    } else {
        count = cb->size + cb->head - cb->tail;
    }
    
    pthread_mutex_unlock(&cb->lock);
    return count;
}

/********************* SPI Helper Functions *********************/

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

/********************* CH432T Register Access *********************/

static uint8_t read_register(DFRobot_CH432T *handle, uint8_t reg) {
    pthread_mutex_lock(&ch432t_spi_lock);

    uint8_t buf[2];
    buf[0] = 0xFD & ((reg + handle->portnum * 0x08) << CH432T_REG_SHIFT);
    buf[1] = 0xFF;

    digitalWrite(handle->cs_pin, LOW); 
    wiringPiSPIDataRW(handle->portnum, buf, 2);
    digitalWrite(handle->cs_pin, HIGH);
      
    pthread_mutex_unlock(&ch432t_spi_lock);
    delay(1);
    return buf[1];
}

static void read_register_buffer(DFRobot_CH432T *handle, uint8_t reg, uint8_t *rx_buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        rx_buf[i] = read_register(handle, reg);
    }
}

static void write_register_single(DFRobot_CH432T *handle, uint8_t reg, uint8_t data) {
    pthread_mutex_lock(&ch432t_spi_lock);

    uint8_t buf[2];
    buf[0] = 0x02 | ((reg + handle->portnum * 0x08) << CH432T_REG_SHIFT);
    buf[1] = data;

    digitalWrite(handle->cs_pin, LOW);
    wiringPiSPIDataRW(handle->portnum, buf, 2);
    digitalWrite(handle->cs_pin, HIGH);
    
    pthread_mutex_unlock(&ch432t_spi_lock);
   delay(1); 
}

static void write_register_buffer(DFRobot_CH432T *handle, uint8_t reg, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        write_register_single(handle, reg, data[i]);
    }
}

static void reg_bit_update(DFRobot_CH432T *handle, uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t temp = read_register(handle, reg);
    temp &= (uint8_t)(~mask);
    temp |= (uint8_t)(value & mask);
    write_register_single(handle, reg, temp);
}

/********************* Interrupt Service Routine *********************/

static void ch432t_isr(void *userdata) {
    DFRobot_CH432T *handle = (DFRobot_CH432T *)userdata;
    
    // Read LSR to check if data is available
    uint8_t lsr = read_register(handle, CH432T_LSR_REG);
    
    while (lsr & CH432T_LSR_DR_BIT) {
        // Read data from RBR
        uint8_t data = read_register(handle, CH432T_RBR_REG);
        
        // Store in circular buffer
        if (!circular_buffer_write(&handle->rx_buffer, data)) {
            // Buffer overflow - handle as needed
            break;
        }
        
        // Signal that data is available
        handle->data_available = true;
        pthread_cond_signal(&handle->data_cond);
        
        // Check if more data is available
        lsr = read_register(handle, CH432T_LSR_REG);
    }
}

/********************* CH432T Core Functions *********************/

DFRobot_CH432T *CH432T_create(int portnum, unsigned int baudrate, CH432T_DataBits bytesize,
                             CH432T_Parity parity, CH432T_StopBits stopbits) {
    DFRobot_CH432T *handle = (DFRobot_CH432T*)calloc(1, sizeof(DFRobot_CH432T));
    handle->spi_fd = -1;
    handle->cs_pin = 8; // default
    handle->portnum = portnum;
    handle->baudrate = baudrate;
    handle->bytesize = bytesize;
    handle->parity = parity;
    handle->stopbits = stopbits;
    handle->is_open = false;
    handle->interrupt_pin = 25; // Default interrupt pin (change as needed)
    
    // Initialize circular buffer (1024 bytes buffer size)
    circular_buffer_init(&handle->rx_buffer, 1024);
    handle->data_available = false;
    pthread_cond_init(&handle->data_cond, NULL);
    
    return handle;
}

void CH432T_destroy(DFRobot_CH432T *handle) {
    if (!handle) return;
    
    if (handle->is_open) {
        CH432T_close(handle);
    }
    
    circular_buffer_free(&handle->rx_buffer);
    pthread_cond_destroy(&handle->data_cond);
    free(handle);
}

static void ch432t_reconfigure_port(DFRobot_CH432T *handle) {
    // word length
    uint8_t cflag = 0;
    switch (handle->bytesize) {
        case DATABITS_5: cflag |= CH432T_LCR_WORD_LEN_5; break;
        case DATABITS_6: cflag |= CH432T_LCR_WORD_LEN_6; break;
        case DATABITS_7: cflag |= CH432T_LCR_WORD_LEN_7; break;
        case DATABITS_8: cflag |= CH432T_LCR_WORD_LEN_8; break;
        default: cflag |= CH432T_LCR_WORD_LEN_8; break;
    }

    // stop bits
    if (handle->stopbits == STOPBITS_ONE) {
        cflag |= CH432T_STOPBIT_1;
    } else {
        cflag |= CH432T_STOPBIT_2;
    }

    // parity
    switch (handle->parity) {
        case PARITY_NONE: break;
        case PARITY_ODD: cflag |= (CH432T_LCR_PARITY_EN_BIT | CH432T_CHECKBIT_ODD); break;
        case PARITY_EVEN: cflag |= (CH432T_LCR_PARITY_EN_BIT | CH432T_CHECKBIT_EVEN); break;
        case PARITY_MARK: cflag |= (CH432T_LCR_PARITY_EN_BIT | CH432T_CHECKBIT_MARK); break;
        case PARITY_SPACE: cflag |= (CH432T_LCR_PARITY_EN_BIT | CH432T_CHECKBIT_SPACE); break;
    }

    // write LCR
    write_register_single(handle, CH432T_LCR_REG, cflag);

    // reset FIFOs, enable FIFOs, set trigger to 8
    uint8_t fcr_val = (CH432T_FCR_FIFO_BIT | CH432T_FCR_RXRESET_BIT | CH432T_FCR_TXRESET_BIT | CH432T_FCR_RECVTG_LEN_8);
    write_register_single(handle, CH432T_FCR_REG, fcr_val);

    // enable interrupts
    uint8_t ier_val = (CH432T_IER_RDI_BIT | CH432T_IER_RLSI_BIT | CH432T_IER_MSI_BIT);
    write_register_single(handle, CH432T_IER_REG, ier_val);

    // enable RTS, OUT2, AFE
    uint8_t mcr_val = (CH432T_MCR_RTS_BIT | CH432T_MCR_OUT2 | CH432T_MCR_AFE);
    write_register_single(handle, CH432T_MCR_REG, mcr_val);

    // set baud rate
    double base_clock = (double)CH432T_CLOCK_FREQUENCY / 12.0; // CK2X=0
    bool ck2x = false;
    if (handle->baudrate > 115200) {
        ck2x = true;
        base_clock = (double)CH432T_CLOCK_FREQUENCY / 12.0 * 24.0;
    }
    
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
    double divisor = (base_clock / 16.0) / (double)handle->baudrate;
    uint16_t mode = (uint16_t)(divisor + 0.5);
    write_register_single(handle, CH432T_DLL_REG, (uint8_t)(mode & 0xFF));
    write_register_single(handle, CH432T_DLH_REG, (uint8_t)((mode >> 8) & 0xFF));

    // restore LCR
    write_register_single(handle, CH432T_LCR_REG, old_lcr);
}

bool CH432T_open(DFRobot_CH432T *handle) {
    int ret = 0;
    if (!handle){
        printf("handle not created\n");
        return false;
    }

    if (handle->is_open) {
        return true;
    }

    // Initialize wiringPi
    while(wiringPiSetupGpio()!=0); 
    printf("wiringPiSetupGpio library is initialized\n");
    if((ret=wiringPiSPISetup(handle->portnum, SPI_SPEED)) == -1 ){
        printf("wiringPiSPISetup failed for port %d\n", handle->portnum);
    }

    printf("/dev/spidev%d is selected\n", handle->portnum);
 
    digitalWrite(handle->cs_pin, HIGH);
    delay(100);

    // Open SPI
    handle->spi_fd = spi_open_device(SPI_DEVICE, SPI_SPEED);
    if (handle->spi_fd < 0) {
        fprintf(stderr, "Failed to open SPI device %s\n", SPI_DEVICE);
        return false;
    }

    // Setup interrupt pin
    pinMode(handle->interrupt_pin, INPUT);
    pullUpDnControl(handle->interrupt_pin, PUD_UP);
    
    // Test communication
    write_register_single(handle, CH432T_SCR_REG, 0x66);
    uint8_t scr = read_register(handle, CH432T_SCR_REG);
    if (scr != 0x66) {
        printf("scr reads 0x%x\n",scr);
        fprintf(stderr, "Failed to open CH432T port: check connections, etc.\n");
        spi_close_device(handle->spi_fd);
        handle->spi_fd = -1;
        return false;
    }

    // Configure port
    ch432t_reconfigure_port(handle);
    
    // Set up interrupt handler
    if (wiringPiISR(handle->interrupt_pin, INT_EDGE_FALLING, ch432t_isr) < 0) {
        fprintf(stderr, "Failed to set up interrupt handler\n");
        spi_close_device(handle->spi_fd);
        handle->spi_fd = -1;
        return false;
    }

    handle->is_open = true;
    return true;
}

void CH432T_close(DFRobot_CH432T *handle) {
    if (!handle || !handle->is_open) return;
    
    // Disable interrupts first
    reg_bit_update(handle, CH432T_IER_REG, CH432T_IER_RDI_BIT, 0);
    
    // Set low power mode
    reg_bit_update(handle, CH432T_IER_REG, CH432T_IER_LOWPOWER_BIT, CH432T_LOW_POWER_MODE);

    // Close SPI
    spi_close_device(handle->spi_fd);
    handle->spi_fd = -1;
    handle->is_open = false;
}

void CH432T_set_baudrate(DFRobot_CH432T *handle, unsigned int baud) {
    if (!handle || !handle->is_open) return;
    handle->baudrate = baud;
    ch432t_reconfigure_port(handle);
}

void CH432T_reset_output_buffer(DFRobot_CH432T *handle) {
    if (!handle || !handle->is_open) return;
    reg_bit_update(handle, CH432T_FCR_REG, CH432T_FCR_TXRESET_BIT, CH432T_FCR_TXRESET_BIT);
}

void CH432T_reset_input_buffer(DFRobot_CH432T *handle) {
    if (!handle || !handle->is_open) return;
    reg_bit_update(handle, CH432T_FCR_REG, CH432T_FCR_RXRESET_BIT, CH432T_FCR_RXRESET_BIT);
}

int CH432T_read(DFRobot_CH432T *handle, uint8_t *buf, size_t size, unsigned int timeout_ms) {
    if (!handle || !handle->is_open) return -1;
    if (size == 0) return 0;

    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    size_t read_count = 0;
    
    while (read_count < size) {
        // Check if data is immediately available
        if (circular_buffer_read(&handle->rx_buffer, &buf[read_count])) {
            read_count++;
            continue;
        }
        
        // No data available, wait with timeout
        if (timeout_ms == 0) {
            break; // Non-blocking mode
        }
        
        // Calculate remaining time
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        unsigned long elapsed_ms = (unsigned long)((now.tv_sec - start.tv_sec) * 1000UL)
                                  + (unsigned long)((now.tv_nsec - start.tv_nsec) / 1000000UL);
        
        if (elapsed_ms >= timeout_ms) {
            break; // Timeout
        }
        
        // Wait for data with remaining timeout
        struct timespec abs_timeout;
        clock_gettime(CLOCK_MONOTONIC, &abs_timeout);
        abs_timeout.tv_nsec += (timeout_ms - elapsed_ms) * 1000000UL;
        if (abs_timeout.tv_nsec >= 1000000000UL) {
            abs_timeout.tv_sec += abs_timeout.tv_nsec / 1000000000UL;
            abs_timeout.tv_nsec %= 1000000000UL;
        }
        
        pthread_mutex_lock(&handle->rx_buffer.lock);
        if (!handle->data_available) {
            pthread_cond_timedwait(&handle->data_cond, &handle->rx_buffer.lock, &abs_timeout);
        }
        handle->data_available = false;
        pthread_mutex_unlock(&handle->rx_buffer.lock);
    }

    return (int)read_count;
}

int CH432T_write(DFRobot_CH432T *handle, const uint8_t *data, size_t size) {
    if (!handle || !handle->is_open) return -1;

    for (size_t i = 0; i < size; i++) {
        write_register_single(handle, CH432T_THR_REG, data[i]);
    }

    return (int)size;
}

void CH432T_set_low_power_mode(DFRobot_CH432T *handle, uint8_t mode) {
    if (!handle || !handle->is_open) return;
    reg_bit_update(handle, CH432T_IER_REG, CH432T_IER_LOWPOWER_BIT, mode);
}

void CH432T_set_sleep_mode(DFRobot_CH432T *handle, uint8_t mode) {
    if (!handle || !handle->is_open) return;
    reg_bit_update(handle, CH432T_IER_REG, CH432T_IER_SLEEP_BIT, mode);
}

void CH432T_set_interrupts(DFRobot_CH432T *handle, uint8_t mask) {
    if (!handle || !handle->is_open) return;
    reg_bit_update(handle, CH432T_IER_REG, 
                  CH432T_IER_RDI_BIT | CH432T_IER_THRI_BIT | CH432T_IER_RLSI_BIT | CH432T_IER_MSI_BIT,
                  mask);
}

uint8_t CH432T_get_line_status(DFRobot_CH432T *handle) {
    if (!handle || !handle->is_open) return 0;
    return read_register(handle, CH432T_LSR_REG);
}

uint8_t CH432T_get_modem_status(DFRobot_CH432T *handle) {
    if (!handle || !handle->is_open) return 0;
    return read_register(handle, CH432T_MSR_REG);
}

/*********************************************************************
 *                   Main Program with Example Usage                  *
 *********************************************************************/

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

int main() {
    // Initialize CH432T with 9600 baud, 8 data bits, no parity, 1 stop bit
    DFRobot_CH432T *ch432 = CH432T_create(CH432T_PORT_1, 9600, DATABITS_8, PARITY_NONE, STOPBITS_ONE);
    
    if (!CH432T_open(ch432)) {
        printf("Failed to open CH432T device.\n");
        CH432T_destroy(ch432);
        return -1;
    }
    printf("CH432T port opened successfully.\n");

    // Enable all interrupts
    CH432T_set_interrupts(ch432, CH432T_IER_RDI_BIT | CH432T_IER_RLSI_BIT | CH432T_IER_MSI_BIT);

    // Prepare Modbus RTU packet
    uint8_t modbus_packet[8];
    uint8_t read_buf[100];
    uint16_t crc;
    int len;
    uint8_t command;

    while(1) {
        // Clear any pending input
        while ((command = getchar()) != '\n' && command != EOF);

        printf("Enter Command ### \n START : 0x02 \n STOP : 0x01 \n REVERSE : 0x20\n FORWARD : 0x10\n-> \n");
        scanf("%x", &command);
 
        command |= 0x08;  // Add enable bit

        // Prepare Modbus command
        modbus_packet[0] = SLAVE_ID;          // Slave address
        modbus_packet[1] = FUNC_WRITE_SINGLE_REGISTER;  // Function code
        modbus_packet[2] = (REGISTER_LOGIC_COMMAND >> 8) & 0xFF;  // Register high byte
        modbus_packet[3] = REGISTER_LOGIC_COMMAND & 0xFF;          // Register low byte
        modbus_packet[4] = 0x00;             // Data high byte
        modbus_packet[5] = command;          // Data low byte (command)

        // Calculate CRC
        crc = calculate_crc16(modbus_packet, 6);
        modbus_packet[6] = crc & 0xFF;       // CRC low byte
        modbus_packet[7] = (crc >> 8) & 0xFF; // CRC high byte

        // Send Modbus packet over CH432T
        CH432T_write(ch432, modbus_packet, sizeof(modbus_packet));
        printf("Modbus command sent to VFD.\n");
        
        // Delay to allow device to process
        delay(100);

        // Read response (using interrupt-based read with 1 second timeout)
        memset(read_buf, 0, sizeof(read_buf));
        len = CH432T_read(ch432, read_buf, sizeof(read_buf), 1000);
        
        if (len > 0) {
            printf("Received %d bytes: ", len);
            for (int i = 0; i < len; i++) {
                printf("%02X ", read_buf[i]);
            }
            printf("\n");
            
            // Verify CRC of response
            if (len >= 4) {  // Minimum valid Modbus RTU response is 4 bytes
                uint16_t received_crc = (read_buf[len-1] << 8) | read_buf[len-2];
                uint16_t calculated_crc = calculate_crc16(read_buf, len-2);
                
                if (received_crc == calculated_crc) {
                    printf("CRC check passed\n");
                } else {
                    printf("CRC check failed (expected %04X, got %04X)\n", calculated_crc, received_crc);
                }
            }
        } else {
            printf("No data received or read timed out.\n");
        }
    }
    
    // Clean up (though we never get here in this example)
    CH432T_close(ch432);
    CH432T_destroy(ch432);
    return 0;
}
