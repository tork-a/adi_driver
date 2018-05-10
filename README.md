# adi_driver [![Build Status](https://travis-ci.org/tork-a/adi_driver.svg?branch=master)](https://travis-ci.org/tork-a/adi_driver)

This package WILL contains ROS driver nodes for Analog Devices(ADI) sensor
products mainly communicate by SPI(Serial Periferal Interface).

Currently supported devices are:

- [ADIS16490](http://www.analog.com/en/products/mems/inertial-measurement-units/adis16490.html)
  - Wide Dynamic Range Mini MEMS IMU


  
You need a SPI interface on your PC to communicate with device. This
package supports
[USB SPI click](https://www.mikroe.com/usb-spi-click)
as the USB-SPI converter.


### Tips

You need to remove the jumper block on ``Power link`` pins to provide
3.3V for the device.

You need to add your user to dialout group to acces /dev/ttyACM* .

``` $ sudo adduser your_user_name dialout ```

If it takes several seconds until /dev/ttyACM* available, you need to
uninstall modemmanager as:

``` $ sudo apt remove modemmanager ```

## ADIS16490

### Overview

[ADIS16490](http://www.analog.com/en/products/mems/inertial-measurement-units/adis16490.html)
is a complete inertial system that includes a triaxis gyroscope and a
triaxis accelerometer.



You can use
[Breakout board](http://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-ADIS16470.html)
for easy use.

### Connection

<div align="center">
  <img src="doc/ADIS16470_Connection.jpg" width="60%"/>
</div>

You need to build a flat cable to connect the USB-clip and the
ADIS16490 breakout board. The picture shows a implementation.

