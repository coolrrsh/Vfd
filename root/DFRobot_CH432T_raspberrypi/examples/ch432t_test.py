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
    try:
        # Write the command to the control register
        master.execute(slave_addr, cst.WRITE_SINGLE_REGISTER, VFD_CONTROL_REGISTER, output_value=command)
        print(f"Command {hex(command)} sent successfully.")
    except Exception:
        # Do nothing in case of failure (error will not be printed)
        pass

def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ['start', 'stop', 'forward', 'reverse', 'reset']:
        print("Usage: python ch432t_vfd_control_demo.py [start|stop|forward|reverse|reset]")
        sys.exit(1)

    # Initialize Modbus RTU Master
    master = modbus_rtu.RtuMaster(ser)
    master.open()
    master.set_timeout(5.0)
    master.set_verbose(True)

    # Determine the command based on user input
    command = None
    if sys.argv[1] == "start":
        command = START_COMMAND
        print("Command: Start")
    elif sys.argv[1] == "stop":
        command = STOP_COMMAND
        print("Command: Stop")
    elif sys.argv[1] == "forward":
        command = FORWARD_COMMAND
        print("Command: Forward")
    elif sys.argv[1] == "reverse":
        command = REVERSE_COMMAND
        print("Command: Reverse")
    elif sys.argv[1] == "reset":
        command = RESET_COMMAND
        print("Command: Reset")
    
    if command is not None:
        send_vfd_command(master, VFD_SLAVE_ID, command)

    # Close the Modbus connection
    master.close()

if __name__ == "__main__":
    main()
