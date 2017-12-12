#ifndef __ADIS16470_H__
#define __ADIS16470_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include "ros/ros.h"

namespace adis16470
{
class IMU
{
public:
  //! File descripter for USB-ISS
  int fd_;
  //! Saved terminal config
  struct termios defaults_;

  // Gyro sensor(x, y, z)
  double gyro[3];
  // Acceleration sensor(x, y, z)
  double accl[3];
  // Magnetic sensor(x, y, z)
  double magn[3];
  
  IMU();
  int openPort(const std::string device);
  void closePort();
  void printInfo(void);

  int get_product_id(short& data);
  int get_seq_count(short& data);
  int update(void);
  int update_burst(void);
  short read_address(char address);
  
};

}

#endif
