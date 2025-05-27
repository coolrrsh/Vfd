# Modbus RS485 Communication (Master-Slave) Example

This project demonstrates Modbus RTU communication over RS485 between a Raspberry Pi (acting as a **Modbus master**) and a PC (acting as a **Modbus slave**) connected via a USB-to-RS485 adapter.

## Overview

The slave application is written in C using the `libmodbus` library and runs on a PC. The Raspberry Pi runs a separate master application (`vfd_app`) that communicates with the slave device using RS485. This setup is useful for developing and testing Modbus-based industrial control systems.

## Setup and Execution

1. **Install Dependencies**  
   On the PC (slave side), ensure that the required Modbus library is installed:

   ```bash
   sudo apt-get install libmodbus
   gcc modbus_slave_usb_rs485.c  -lmodbus -o vfd_slave
   ./vfd_slave

  On the Raspberry Pi,
  ```bash
   ./vfd_app

