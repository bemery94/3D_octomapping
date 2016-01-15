#include "myahrs_plus_api.hpp"
#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sstream>
#include <string>

//access methods within the WithRobot API -> check readme for details
using namespace WithRobot;

//TODO make these params?
//char* serial_device = "/dev/ttyACM0";
std::string serial_device = "/dev/ttyACM0";
int baudrate = 115200;

static const char* DIVIDER = "1";  // 100 Hz

//ros variables
sensor_msgs::Imu imu_data_;

//ros publisher
const std::string imu_topic_ = "myahrs_imu";
ros::Publisher myahrs_pub;
