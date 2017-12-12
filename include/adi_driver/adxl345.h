#ifndef __ADXL345_H__
#define __ADXL345_H__

#include <termios.h>

namespace adxl345
{
class Imu
{
public: 
  //! File descripter for USB-ISS
  int fd_;
  //! Saved terminal config
  struct termios defaults_;
  // Acceleration x,y,z
  double accl[3];
  
  Imu();
  char read_address(char address);
  short read_short(char address);
  char write_address(char address, char data);
  int open_device(const std::string device);
  void close_device();
  int get_product_id(unsigned char& data);
  int get_seq_count(short& data);
  int update(void);
};

}
#endif
