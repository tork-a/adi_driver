#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "adi_driver/adxl345.h"

using namespace std;

class Adxl345Node
{
public:
  adxl345::Imu imu_;
  ros::NodeHandle node_handle_;
  ros::Publisher imu_data_pub_;
  std::string device_;
  string frame_id_;

  Adxl345Node(ros::NodeHandle nh)
    : node_handle_(nh)
  {
    // Read parameters
    node_handle_.param("device", device_, string("/dev/ttyACM0"));
    node_handle_.param("frame_id", frame_id_, string("imu"));
    
    imu_data_pub_ = node_handle_.advertise<sensor_msgs::Imu>("data_raw", 100);
  }

  ~Adxl345Node()
  {
    imu_.close_device();
  }

  /**
   * @brief Check if the device is opened
   */
  bool is_opened(void)
  {
    return (imu_.fd_ >= 0);
  }
  /**
   * @brief Open IMU device file
   */
  bool open(void)
  {
    // Open device file
    if (imu_.open_device(device_) < 0)
    {
      ROS_ERROR("Failed to open device %s", device_.c_str());
    }
    // Wait 10ms for SPI ready
    usleep(10000);
    unsigned char pid = 0;
    imu_.get_product_id(pid);
    ROS_INFO("Product ID: %0x\n", pid);
  }
  int publish_imu_data()
  {
    sensor_msgs::Imu data;
    data.header.frame_id = frame_id_;
    data.header.stamp = ros::Time::now();

    // Linear acceleration
    data.linear_acceleration.x = imu_.accl[0];
    data.linear_acceleration.y = imu_.accl[1];
    data.linear_acceleration.z = imu_.accl[2];

    // Orientation (quarternion)
    data.orientation.x = 0;
    data.orientation.y = 0;
    data.orientation.z = 0;
    data.orientation.w = 0;

    imu_data_pub_.publish(data);
  }
  
  bool spin()
  {
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
      if (imu_.update() == 0)
      {
        publish_imu_data();
      }
      else
      {
        ROS_ERROR("Cannot update");
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
    return true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "adxl345_node");
  ros::NodeHandle nh("~");
  Adxl345Node node(nh);

  node.open();
  node.spin();
  return(0);
}
