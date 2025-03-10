# DFRobot_CH432T
- [English Version](./README.md)

ch432t树莓派SPI转modbus扩展板, 解放树莓派串口, 使SPI转换为两个modbus串口

![产品实物图](./resources/images/CH432T.png)


## 产品链接 (https://www.dfrobot.com.cn/)
    SKU: DFR0824


## 目录

* [概述](#概述)
* [库安装](#库安装)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [创作者](#创作者)


## 概述

* ch432t树莓派SPI转modbus扩展板驱动


## 库安装

首先本库为ch432t树莓派SPI转modbus扩展板驱动, 需要使用树莓派的spidev0.0, 需要提前使能树莓派SPI接口。

本库为方便使用, 将外部接口保持和modbus_tk使用serial方式相似, 故此还依赖serial.serialutil, 不过此工具库, 树莓派一般默认都有, 若无特殊情况则可忽略。

本库示例使用modbus_tk, 使用本库前先检测树莓派是否成功导入modbus_tk, 若导入失败, 请通过以下命令安装modbus_tk库:
python2: pip install modbus_tk
python3: pip3 install modbus_tk

使用库, 首先下载库文件, 将其粘贴到指定的目录中, 然后打开Examples文件夹并在该文件夹中运行演示。


## 方法

```python

  '''!
    @brief Initialize port
    @note 当硬件通信出错, 或者配置有误将抛出异常
  '''
  def open(self):

  '''!
    @brief 关闭端口, 关闭该串口的内部基准时钟，从而使该串口进入低功耗状态
  '''
  def close(self):

  '''!
    @brief 读串口数据
    @param size 读串口数据的长度
    @return 读到的串口数据
  '''
  def read(self, size):

  '''!
    @brief 写串口数据
    @param data 写入串口的数据
  '''
  def write(self, data):

```


## 兼容性

* RaspberryPi 版本

| Board        | Work Well | Work Wrong | Untested | Remarks |
| ------------ | :-------: | :--------: | :------: | ------- |
| RaspberryPi2 |           |            |    √     |         |
| RaspberryPi3 |           |            |    √     |         |
| RaspberryPi4 |     √     |            |          |         |

* Python 版本

| Python  | Work Well | Work Wrong | Untested | Remarks |
| ------- | :-------: | :--------: | :------: | ------- |
| Python2 |     √     |            |          |         |
| Python3 |     √     |            |          |         |


## 历史

- 2021/12/30 - 1.0.0 版本
- 2022/03/30 - 1.0.1 版本
- 2022/09/24 - 1.0.2 版本


## 创作者

Written by qsjhyy(yihuan.huang@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))

