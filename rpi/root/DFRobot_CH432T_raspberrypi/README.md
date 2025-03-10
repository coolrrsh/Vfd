# DFRobot_CH432T
- [中文版](./README_CN.md)

This CH432t Raspberry Pi SPI to modbus expansion board is designed to convert Pi's SPI to two modbus serial ports.

![产品实物图](./resources/images/CH432T.png)


## Product Link (https://www.dfrobot.com/)
    SKU: DFR0824


## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)


## Summary

* CH432T Raspberry Pi SPI to modbus expansion board


## Installation

1. This library is for the CH432T Raspberry Pi SPI to modbus expansion board. The Raspberry Pi spidev0.0 is required, and please enable the Raspberry Pi SPI interface before use.

2. This library keeps the serial using method of external interface be the same as that of modbus_tk for easy use. So it requires serial.serialutil. Raspberry Pi is generally default to have the tool library. Ignore it if there is no special requirements.

3. This library example uses modbus_tk. Check whether the Raspberry Pi has successfully imported modbus_tk before use, if not, install modbus_tk library by the following command:

    python2: pip install modbus_tk <br/>
    python3: pip3 install modbus_tk  <br/>

4. Download the library file before use, paste them into the specified directory, then open the Examples folder and run the demo in the folder.


## Methods

```python

  '''!
    @brief Initialize port
    @note Exceptions will be thrown when a hardware communication or config error occurs
  '''
  def open(self):

  '''!
    @brief Close the serial port and the internal reference clock of it, so as to make the serial port enter low-power status
  '''
  def close(self):

  '''!
    @brief Read serial data
    @param size Read length of serial data
    @return The read serial data
  '''
  def read(self, size):

  '''!
    @brief Write serial data
    @param data The data to be written into serial
  '''
  def write(self, data):

```


## Compatibility

* RaspberryPi Version

| Board        | Work Well | Work Wrong | Untested | Remarks |
| ------------ | :-------: | :--------: | :------: | ------- |
| RaspberryPi2 |           |            |    √     |         |
| RaspberryPi3 |           |            |    √     |         |
| RaspberryPi4 |     √     |            |          |         |

* Python Version

| Python  | Work Well | Work Wrong | Untested | Remarks |
| ------- | :-------: | :--------: | :------: | ------- |
| Python2 |     √     |            |          |         |
| Python3 |     √     |            |          |         |


## History

- 2021/12/30 - Version 1.0.0 released.
- 2022/03/30 - Version 1.0.1 released.
- 2022/09/24 - Version 1.0.2 released.


## Credits

Written by qsjhyy(yihuan.huang@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))

