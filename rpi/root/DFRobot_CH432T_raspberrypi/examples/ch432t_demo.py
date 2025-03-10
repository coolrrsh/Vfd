# -*- coding: utf_8 -*-
'''!
  @file  ch432t_vfd_control_demo.py
  @brief  This is a demo to send start/stop commands to a VFD using Modbus RTU.
  @n  Ensure the Modbus interfaces are correctly connected and the slave address and register settings match your VFD.
  @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license  The MIT License (MIT)
  @author  [qsjhyy](yihuan.huang@dfrobot.com)
  @version  V1.0
  @date  2022-09-24
  @url  https://github.com/DFRobot/DFRobot_CH432T
'''

from __future__ import print_function
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import time

import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
from DFRobot_CH432T import *   # Import DFRobot_CH432T to replace importing serial library

# Modbus slave settings for the VFD
VFD_SLAVE_ID = 100                # VFD Modbus slave ID
VFD_CONTROL_REGISTER = 0x2000     # Control register address (0x2000 in hex corresponds to 8192 in decimal)

# Control commands (defined as constants)
RESET_COMMAND = 0x0000       # Reset command (neutral state)
START_COMMAND = 0x0002       # Start command without direction
FORWARD_COMMAND = 0x0012     # Start + Forward command
REVERSE_COMMAND = 0x0022     # Start + Reverse command
STOP_COMMAND = 0x0001        # Stop command

PORT_1 = "CH432T_PORT_1"          # Use the same port for both master and slave
ser = DFRobot_CH432T(port=PORT_1, baudrate=9600, bytesize=8, parity='N', stopbits=1)

def send_vfd_command(master, slave_addr, command):
    """
    Sends a command to the VFD control register.
    :param master: Modbus master instance
    :param slave_addr: Modbus slave address
    :param command: Command to send to the VFD (Start/Stop/Forward/Reverse)
    """
    print(f"Debug: Attempting to send command to VFD at slave address {slave_addr}...")
    try:
        print(f"Debug: Executing Modbus command to write {hex(command)} to register {hex(VFD_CONTROL_REGISTER)}...")
        # Write the command to the control register
        master.execute(slave_addr, cst.WRITE_SINGLE_REGISTER, VFD_CONTROL_REGISTER, output_value=command)
        print(f"Debug: Command sent successfully: {hex(command)}")
    except Exception as e:
        print(f"Debug: Failed to send command: {e}")

def main():
    # Debug: Verify input parameters
    print(f"Debug: Command-line arguments: {sys.argv}")
    
    if len(sys.argv) != 2 or sys.argv[1] not in ['start', 'stop', 'forward', 'reverse', 'reset']:
        print("Usage: python ch432t_vfd_control_demo.py [start|stop|forward|reverse|reset]")
        sys.exit(1)

    # Initialize Modbus RTU Master
    print("Debug: Initializing the master port...")
    print(f"Debug: Using serial port: {ser.name}")
    
    master = modbus_rtu.RtuMaster(ser)
    print("Debug: Opening Modbus master connection...")
    master.open()
    print(f"Debug: Modbus master connection opened with timeout {master.get_timeout()} seconds")
    
    master.set_timeout(10.0)
    master.set_verbose(True)
    print("Debug: Modbus master settings: timeout = 10.0s, verbose mode = True")

    # Determine the command based on user input
    command = None
    if sys.argv[1] == "start":
        print("Debug: Preparing to send Start command to the VFD...")
        command = START_COMMAND
    elif sys.argv[1] == "stop":
        print("Debug: Preparing to send Stop command to the VFD...")
        command = STOP_COMMAND
    elif sys.argv[1] == "forward":
        print("Debug: Preparing to send Forward command to the VFD...")
        command = FORWARD_COMMAND
    elif sys.argv[1] == "reverse":
        print("Debug: Preparing to send Reverse command to the VFD...")
        command = REVERSE_COMMAND
    elif sys.argv[1] == "reset":
        print("Debug: Preparing to send Reset command to the VFD...")
        command = RESET_COMMAND
    
    if command is not None:
        send_vfd_command(master, VFD_SLAVE_ID, command)

    # Close the Modbus connection
    print("Debug: Closing Modbus connection...")
    master.close()
    print("Debug: Modbus connection closed.")

if __name__ == "__main__":
    print("Debug: Starting the program...")
    main()
    print("Debug: Program finished.")
