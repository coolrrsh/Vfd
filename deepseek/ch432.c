#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

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

#define CH432T_IER_RDI_BIT (1 << 0)
#define CH432T_IER_THRI_BIT (1 << 1)
#define CH432T_IER_RLSI_BIT (1 << 2)
#define CH432T_IER_MSI_BIT (1 << 3)
#define CH432T_IER_RESET_BIT (1 << 7)
#define CH432T_IER_LOWPOWER_BIT (1 << 6)
#define CH432T_IER_SLEEP_BIT (1 << 5)
#define CH432T_IER_CK2X_BIT (1 << 5)
#define CH432T_LOW_POWER_MODE CH432T_IER_LOWPOWER_BIT
#define CH432T_SLEEP_MODE CH432T_IER_SLEEP_BIT
#define CH432T_STANDARD_MODE 0

#define CH432T_FCR_FIFO_BIT (1 << 0)
#define CH432T_FCR_RXRESET_BIT (1 << 1)
#define CH432T_FCR_TXRESET_BIT (1 << 2)
#define CH432T_FCR_RXLVL_BIT (0x03 << 6)
#define CH432T_FCR_RECVTG_LEN_1 (0x00 << 6)
#define CH432T_FCR_RECVTG_LEN_4 (0x01 << 6)
#define CH432T_FCR_RECVTG_LEN_8 (0x02 << 6)
#define CH432T_FCR_RECVTG_LEN_14 (0x03 << 6)

#define CH432T_LCR_LENGTH_BIT (0x03 << 0)
#define CH432T_LCR_STOPLEN_BIT (1 << 2)
#define CH432T_LCR_PARITY_EN_BIT (1 << 3)
#define CH432T_LCR_PARITY_MODE_BIT (0x03 << 4)
#define CH432T_LCR_TXBREAK_BIT (1 << 6)
#define CH432T_LCR_DLAB_BIT (1 << 7)
#define CH432T_LCR_CONF_MODE_A CH432T_LCR_DLAB_BIT

#define CH432T_MCR_DTR_BIT (1 << 0)
#define CH432T_MCR_RTS_BIT (1 << 1)
#define CH432T_MCR_OUT1 (1 << 2)
#define CH432T_MCR_OUT2 (1 << 3)
#define CH432T_MCR_LOOP_BIT (1 << 4)
#define CH432T_MCR_AFE (1 << 5)

#define CH432T_LSR_DR_BIT (1 << 0)
#define CH432T_LSR_OE_BIT (1 << 1)
#define CH432T_LSR_PE_BIT (1 << 2)
#define CH432T_LSR_FE_BIT (1 << 3)
#define CH432T_LSR_BI_BIT (1 << 4)
#define CH432T_LSR_THRE_BIT (1 << 5)
#define CH432T_LSR_TEMT_BIT (1 << 6)
#define CH432T_LSR_FIFOE_BIT (1 << 7)

#define CH432T_MSR_DCTS_BIT (1 << 0)
#define CH432T_MSR_DDSR_BIT (1 << 1)
#define CH432T_MSR_DRI_BIT (1 << 2)
#define CH432T_MSR_DCD_BIT (1 << 3)
#define CH432T_MSR_CTS_BIT (1 << 4)
#define CH432T_MSR_DSR_BIT (1 << 5)
#define CH432T_MSR_RI_BIT (1 << 6)
#define CH432T_MSR_CD_BIT (1 << 7)

#define CH432T_PORT_1 0
#define CH432T_PORT_2 1

#define CH432T_CLOCK_FREQUENCY 22118400
#define CH432T_FIFO_SIZE 16
#define CH432T_REG_SHIFT 2

#define SPI_CHANNEL 0
#define SPI_SPEED 1000000
#define CS_PIN 8

typedef struct {
    uint8_t reset : 1;
    uint8_t low_power : 1;
    uint8_t slp_ck2x : 1;
    uint8_t reserved : 1;
    uint8_t ie_modem : 1;
    uint8_t ie_lines : 1;
    uint8_t ie_thre : 1;
    uint8_t ie_recv : 1;
} INT_config_reg;

typedef struct {
    uint8_t int_type : 4;
    uint8_t reserved : 2;
    uint8_t fifo_ENS : 2;
} INT_status_reg;

typedef struct {
    uint8_t fifo_EN : 1;
    uint8_t r_fifo_rst : 1;
    uint8_t t_fifo_rst : 1;
    uint8_t reserved : 3;
    uint8_t recv_TG : 2;
} fifo_config_reg;

typedef struct {
    uint8_t word_size : 2;
    uint8_t stop_bit : 1;
    uint8_t parity_EN : 1;
    uint8_t parity_mode : 2;
    uint8_t break_EN : 1;
    uint8_t DLAB : 1;
} lines_config_reg;

typedef struct {
    uint8_t DTR : 1;
    uint8_t RTS : 1;
    uint8_t out1 : 1;
    uint8_t out2 : 1;
    uint8_t loop : 1;
    uint8_t AFE : 1;
    uint8_t reserved : 2;
} modem_config_reg;

typedef struct {
    uint8_t data_ready : 1;
    uint8_t fifo_over : 1;
    uint8_t parity_err : 1;
    uint8_t frame_err : 1;
    uint8_t break_INT : 1;
    uint8_t THR_EN : 1;
    uint8_t t_empty : 1;
    uint8_t r_fifo_err : 1;
} lines_status_reg;

typedef struct {
    uint8_t CTS_change : 1;
    uint8_t DSR_change : 1;
    uint8_t RI_change : 1;
    uint8_t DCD_change : 1;
    uint8_t CTS : 1;
    uint8_t DSR : 1;
    uint8_t RI : 1;
    uint8_t DCD : 1;
} modem_status_reg;

typedef struct {
    int portnum;
    int _cs;
    int _spi;
} DFRobot_CH432T;

void DFRobot_CH432T_init(DFRobot_CH432T *dev, int portnum) {
    dev->portnum = portnum;
    dev->_cs = CS_PIN;
    wiringPiSetup();
    wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED);
    pinMode(dev->_cs, OUTPUT);
    digitalWrite(dev->_cs, HIGH);
}

void _write_reg(DFRobot_CH432T *dev, uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = 0x02 | ((reg + dev->portnum * 0x08) << CH432T_REG_SHIFT);
    buf[1] = data;
    digitalWrite(dev->_cs, LOW);
    wiringPiSPIDataRW(SPI_CHANNEL, buf, 2);
    digitalWrite(dev->_cs, HIGH);
    delay(1);
}

uint8_t _read_reg(DFRobot_CH432T *dev, uint8_t reg) {
    uint8_t buf[2];
    buf[0] = 0xFD & ((reg + dev->portnum * 0x08) << CH432T_REG_SHIFT);
    buf[1] = 0xFF;
    digitalWrite(dev->_cs, LOW);
    wiringPiSPIDataRW(SPI_CHANNEL, buf, 2);
    digitalWrite(dev->_cs, HIGH);
    return buf[1];
}

void set_baudrate(DFRobot_CH432T *dev, int baud) {
    int prescaler = 0;
    int clock_rate = CH432T_CLOCK_FREQUENCY / 12;
    if (baud > 115200) {
        prescaler = CH432T_IER_CK2X_BIT;
        clock_rate *= 24;
    }
    uint8_t reg = CH432T_IER_REG + (dev->portnum == CH432T_PORT_1 ? 0x08 : 0);
    _write_reg(dev, reg, prescaler);

    uint8_t lcr = _read_reg(dev, CH432T_LCR_REG);
    _write_reg(dev, CH432T_LCR_REG, CH432T_LCR_CONF_MODE_A);
    delay(2);

    int mode = (int)(clock_rate / 16 / baud);
    _write_reg(dev, CH432T_DLL_REG, mode & 0xFF);
    _write_reg(dev, CH432T_DLH_REG, (mode >> 8) & 0xFF);

    _write_reg(dev, CH432T_LCR_REG, lcr);
}

int main() {
    DFRobot_CH432T dev;
    DFRobot_CH432T_init(&dev, CH432T_PORT_1);

    _write_reg(&dev, CH432T_SCR_REG, 0x66);
    uint8_t scr = _read_reg(&dev, CH432T_SCR_REG);
    printf("CH432T_SCR_REG = 0x%02X\n", scr);

    set_baudrate(&dev, 115200);

    return 0;
}
