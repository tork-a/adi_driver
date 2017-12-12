#include "adi_driver/adis16470.h"

adis16470::IMU::IMU()
  : fd_(-1)
{
}

/**
 * @brief Open IMU device
 * @param device Device file name (/dev/ttyACM*)
 * @retval 0 Success
 * @retval -1 Failure
 */
int adis16470::IMU::openPort(const std::string device)
{
  struct termios config;

  fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);
  if(fd_ < 0)
  {
    return -1;
  }
  if (tcgetattr(fd_, &defaults_) < 0)
  {
    perror("tcgetattr");
  }
  cfmakeraw(&config);
  if (tcsetattr(fd_, TCSANOW, &config) < 0)
  {
    perror("tcsetattr config");
  }
  // set SPI mode
  unsigned char buff[20];
  buff[0] = 0x5A;
  buff[1] = 0x02; // Set mode command
  buff[2] = 0x93; // Set SPI mode 
  buff[3] = 5; // 1MHz clock speed

  if (write(fd_, buff, 4) < 0)
  {
    ROS_ERROR("write error");
  }
  if (tcdrain(fd_) < 0)
  {
    ROS_ERROR("set_spi_mode tcdrain");
  }
  if (read(fd_, buff, 2) < 0)
  {
    ROS_ERROR("set_spi_mode read");
  }
  // Read back error byte
  if(buff[0] != 0xFF)
  {
    ROS_ERROR("**set_spi_mode: Error setting spi mode!**\n\n");
  }
  return 0;
}

void adis16470::IMU::printInfo()
{
  unsigned char buff[20];

  buff[0] = 0x5A;
  buff[1] = 0x01; // Software return byte

  if (write(fd_, buff, 2) < 0) perror("display_version write");
  if (tcdrain(fd_) < 0) perror("display_version tcdrain");
  if (read(fd_, buff, 3) < 0) perror("display_version read");

  ROS_INFO("USB-ISS Module ID: %u \n", buff[0]);
  ROS_INFO("USB-ISS Software v: %u \n\n", buff[1]);
}

void adis16470::IMU::closePort()
{
  if (tcsetattr(fd_, TCSANOW, &defaults_) < 0)
  {
    perror("tcsetattr defaults");
  }
  close(fd_);
}

/**
 * @param data Product ID (0x4056)
 * @return 0: Success, 1: Failed
 */
int adis16470::IMU::get_product_id(short& pid)
{
  // get product ID
  int r;
  unsigned char buff[20];

  buff[0] = 0x61;
  buff[1] = 0x72;
  buff[2] = 0x00;

  if (write(fd_, buff, 3) < 0)
  {
    perror("write");
  }
  if (tcdrain(fd_) < 0)
  {
    perror("tcdrain");
  }
  if (read(fd_, buff, 3) < 0)
  {
    perror("read");
  }
  buff[0] = 0x61;
  buff[1] = 0x00;
  buff[2] = 0x00;
  if (write(fd_, buff, 3) < 0)
  {
    perror("write");
  }
  if (tcdrain(fd_) < 0)
  {
    perror("tcdrain");
  }
  if (read(fd_, buff, 3) < 0)
  {
    perror("read");
  }
  char b[2] = {buff[2], buff[1]};
  pid = *((short*)b);
  return 0;
}

int adis16470::IMU::get_seq_count(short& data)
{
  return 0;
}

/**
 * @brief Update all information by bust read
 * @return 0: Success, -1: Failed
 */
int adis16470::IMU::update_burst(void)
{
  char buff[64];

  memset(buff, 0, sizeof(buff));
  // 0x6800: Burst read function
  buff[0] = 0x61;
  buff[1] = 0x68;
  buff[2] = 0x00;
  
  if (write(fd_, buff, 24) < 0)
  {
    perror("write");
  }
  if (tcdrain(fd_) < 0)
  {
    perror("tcdrain");
  }
  int size = read(fd_, buff, 30);
  if (size < 0)
  {
    perror("read");
  }
  unsigned short diag_stat;
  unsigned char buff2[2];

  buff2[0] = buff[4];
  buff2[1] = buff[3];
  diag_stat = *((short*)buff2);
  if (diag_stat != 0)
  {
    ROS_WARN("Some error received: diag_stat: %x\n", diag_stat);
  }
  // X_GYRO_OUT
  buff2[0] = buff[6];
  buff2[1] = buff[5];
  gyro[0] = *((short*)buff2) * M_PI / 180 / 10.0;
  // Y_GYRO_OUT
  buff2[0] = buff[8];
  buff2[1] = buff[7];
  gyro[1] = *((short*)buff2) * M_PI / 180 / 10.0;
  // Z_GYRO_OUT
  buff2[0] = buff[10];
  buff2[1] = buff[9];
  gyro[2] = *((short*)buff2) * M_PI / 180 / 10.0;
  // X_ACCL_OUT
  buff2[0] = buff[12];
  buff2[1] = buff[11];
  accl[0] = *((short*)buff2) * 9.8 / 800.0;
  // Y_ACCL_OUT
  buff2[0] = buff[14];
  buff2[1] = buff[13];
  accl[1] = *((short*)buff2) * 9.8 / 800.0;
  // Z_ACCL_OUT
  buff2[0] = buff[16];
  buff2[1] = buff[15];
  accl[2] = *((short*)buff2) * 9.8 / 800.0;

  return 0;
}  

short adis16470::IMU::read_address(char address)
{
  char buff[3];
  
  buff[0] = 0x61;
  buff[1] = address;
  buff[2] = 0x00;
  if (write(fd_, buff, 3) < 0)
  {
    perror("write");
  }
  if (tcdrain(fd_) < 0)
  {
    ROS_ERROR("tcdrain");
  }
  int size = read(fd_, buff, 3);
  if (size !=3)
  {
    perror("read");
  }
  char buff2[2];
  buff2[0] = buff[2];
  buff2[1] = buff[1];
  return *((short*)buff2);
}

/**
 * @brief update gyro and accel in high-precision read
 */
int adis16470::IMU::update(void)
{
  short gyro_out[3], gyro_low[3], accl_out[3], accl_low[3];

  read_address(0x04);
  gyro_low[0] = read_address(0x06);
  gyro_out[0] = read_address(0x08);
  gyro_low[1] = read_address(0x0a);
  gyro_out[1] = read_address(0x0c);
  gyro_low[2] = read_address(0x0e);
  gyro_out[2] = read_address(0x10);
  accl_low[0] = read_address(0x12);
  accl_out[0] = read_address(0x14);
  accl_low[1] = read_address(0x16);
  accl_out[1] = read_address(0x18);
  accl_low[2] = read_address(0x1a);
  accl_out[2] = read_address(0x00);

  // 32bit convert
  for (int i=0; i<3; i++)
  {
    gyro[i] = ((int32_t(gyro_out[i]) << 16) + int32_t(gyro_low[i])) * M_PI / 180.0 / 655360.0;
    accl[i] = ((int32_t(accl_out[i]) << 16) + int32_t(accl_low[i])) * 9.8 / 52428800.0;
  }
  return 0;
}
