#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <modbus.h>

// CH432T Register Definitions
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

// CH432T Configuration Bits
#define CH432T_IER_CK2X_BIT (1 << 5)
#define CH432T_LCR_CONF_MODE_A (1 << 7)
#define CH432T_LCR_8BIT_DATA (0x03)  // 8 data bits
#define CH432T_LCR_1STOP_BIT (0x00)  // 1 stop bit
#define CH432T_LCR_NO_PARITY (0x00)  // No parity

// CH432T Port Definitions
#define CH432T_PORT_1 0
#define CH432T_PORT_2 1

// SPI Configuration
#define SPI_CHANNEL 0
#define SPI_SPEED 1000000
#define CS_PIN 8

// Modbus RTU Configuration
#define VFD_SLAVE_ID 100                // VFD Modbus slave ID
#define VFD_CONTROL_REGISTER 8192       // Control register address (0x2000 in hex corresponds to 8192 in decimal)

// Control Commands
#define RESET_COMMAND 0x0000       // Reset command (neutral state)
#define START_COMMAND 0x0002       // Start command without direction
#define FORWARD_COMMAND 0x0012     // Start + Forward command
#define REVERSE_COMMAND 0x0022     // Start + Reverse command
#define STOP_COMMAND 0x0001        // Stop command

// CH432T Device Structure
typedef struct {
    int portnum;
    int _cs;
    int _spi;
} DFRobot_CH432T;

// Function Prototypes
void DFRobot_CH432T_init(DFRobot_CH432T *dev, int portnum);
void _write_reg(DFRobot_CH432T *dev, uint8_t reg, uint8_t data);
uint8_t _read_reg(DFRobot_CH432T *dev, uint8_t reg);
void set_baudrate(DFRobot_CH432T *dev, int baud);
void set_modbus_rtu_config(DFRobot_CH432T *dev);
void send_vfd_command(DFRobot_CH432T *dev, int slave_addr, uint16_t command);

// Initialize CH432T
void DFRobot_CH432T_init(DFRobot_CH432T *dev, int portnum) {
    dev->portnum = portnum;
    dev->_cs = CS_PIN;
    wiringPiSetup();
    wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED);
    pinMode(dev->_cs, OUTPUT);
    digitalWrite(dev->_cs, HIGH);
}

// Write to CH432T Register
void _write_reg(DFRobot_CH432T *dev, uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = 0x02 | ((reg + dev->portnum * 0x08) << 2);
    buf[1] = data;
    digitalWrite(dev->_cs, LOW);
    wiringPiSPIDataRW(SPI_CHANNEL, buf, 2);
    digitalWrite(dev->_cs, HIGH);
    delay(1);
}

// Read from CH432T Register
uint8_t _read_reg(DFRobot_CH432T *dev, uint8_t reg) {
    uint8_t buf[2];
    buf[0] = 0xFD & ((reg + dev->portnum * 0x08) << 2);
    buf[1] = 0xFF;
    digitalWrite(dev->_cs, LOW);
    wiringPiSPIDataRW(SPI_CHANNEL, buf, 2);
    digitalWrite(dev->_cs, HIGH);
    return buf[1];
}

// Set Baud Rate for CH432T
void set_baudrate(DFRobot_CH432T *dev, int baud) {
    int prescaler = 0;
    int clock_rate = 22118400 / 12;
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

// Set Modbus RTU Configuration (8 data bits, 1 stop bit, no parity)
void set_modbus_rtu_config(DFRobot_CH432T *dev) {
    uint8_t lcr = _read_reg(dev, CH432T_LCR_REG);
    lcr &= ~0x1F;  // Clear the lower 5 bits (data bits, stop bits, parity)
    lcr |= CH432T_LCR_8BIT_DATA | CH432T_LCR_1STOP_BIT | CH432T_LCR_NO_PARITY;
    _write_reg(dev, CH432T_LCR_REG, lcr);
}

// Send VFD Command via Modbus RTU
void send_vfd_command(DFRobot_CH432T *dev, int slave_addr, uint16_t command) {
    // Construct Modbus RTU frame
    uint8_t frame[8];
    frame[0] = slave_addr;                  // Slave address
    frame[1] = 0x06;                        // Function code (Write Single Register)
    frame[2] = (VFD_CONTROL_REGISTER >> 8) & 0xFF; // Register address high byte
    frame[3] = VFD_CONTROL_REGISTER & 0xFF; // Register address low byte
    frame[4] = (command >> 8) & 0xFF;       // Command high byte
    frame[5] = command & 0xFF;              // Command low byte
    uint16_t crc = modbus_rtu_crc(frame, 6); // Calculate CRC
    frame[6] = crc & 0xFF;                  // CRC low byte
    frame[7] = (crc >> 8) & 0xFF;           // CRC high byte

    // Send frame via CH432T SPI
    for (int i = 0; i < 8; i++) {
        _write_reg(dev, CH432T_THR_REG, frame[i]);
    }

    printf("Command 0x%04X sent successfully.\n", command);
}

// Modbus RTU CRC Calculation
uint16_t modbus_rtu_crc(uint8_t *buf, int len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// Main Function
int main(int argc, char *argv[]) {
    if (argc != 2 || (strcmp(argv[1], "start") != 0 && strcmp(argv[1], "stop") != 0 &&
                      strcmp(argv[1], "forward") != 0 && strcmp(argv[1], "reverse") != 0 &&
                      strcmp(argv[1], "reset") != 0)) {
        printf("Usage: %s [start|stop|forward|reverse|reset]\n", argv[0]);
        return 1;
    }

    // Initialize CH432T
    DFRobot_CH432T dev;
    DFRobot_CH432T_init(&dev, CH432T_PORT_1);
    set_baudrate(&dev, 9600);
    set_modbus_rtu_config(&dev);  // Set Modbus RTU configuration

    // Determine the command based on user input
    uint16_t command = 0;
    if (strcmp(argv[1], "start") == 0) {
        command = START_COMMAND;
        printf("Command: Start\n");
    } else if (strcmp(argv[1], "stop") == 0) {
        command = STOP_COMMAND;
        printf("Command: Stop\n");
    } else if (strcmp(argv[1], "forward") == 0) {
        command = FORWARD_COMMAND;
        printf("Command: Forward\n");
    } else if (strcmp(argv[1], "reverse") == 0) {
        command = REVERSE_COMMAND;
        printf("Command: Reverse\n");
    } else if (strcmp(argv[1], "reset") == 0) {
        command = RESET_COMMAND;
        printf("Command: Reset\n");
    }

    if (command != 0) {
        send_vfd_command(&dev, VFD_SLAVE_ID, command);
    }

    return 0;
}
