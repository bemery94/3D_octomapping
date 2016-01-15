#pragma once

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <csignal>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <sys/statvfs.h>

#include <boost/thread.hpp>
#include <queue>

#include "messages.h"
#include "record.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/Int32.h"
#include "std_msgs/UInt8.h"
#include <data_recorder/Fs_space.h>

using namespace Sensor_Box_pkg;

//define
#define B_USER		8
#define SW_LSM		4
#define SW_CAM		16

static const unsigned short CAM_ON = 0x01;
static const unsigned short IMU_ON = 0x02;
static const unsigned short LASER_LSL_ON = 0x04;
static const unsigned short LASER_LSM_ON = 0x08;
static const unsigned short DISK_ON = 0x10;
static const unsigned short ALLSENSORS_ON = 0x1F;

typedef enum device_index__ {CAM, IMU, LASER_LSL, LASER_LSM, DISK_MNTR, NUMOFDEVICES} device_index;

//function prototypes
static void exitSignalHandler(int sigid);
void exitIndication(int sig);
void IMU_cb(const sensor_msgs::ImuConstPtr& msg);
void LSL_cb(const sensor_msgs::LaserScanConstPtr& msg);
void LSM_cb(const sensor_msgs::LaserScanConstPtr& msg);
void CAM_cb(const sensor_msgs::ImageConstPtr& msg);
void TFS_cb(const tf::tfMessageConstPtr& msg);
void DISK_cb(const data_recorder::Fs_spaceConstPtr& msg);
void USR_cb(const std_msgs::UInt8& msg);
void IMU_timer_cb(const ros::TimerEvent& event);
void LSL_timer_cb(const ros::TimerEvent& event);
void LSM_timer_cb(const ros::TimerEvent& event);
void CAM_timer_cb(const ros::TimerEvent& event);
void DISK_timer_cb(const ros::TimerEvent& event);

void initialise();
void user_io();
void check_input();
void output_status();
void process();
void monitor();
std::string get_curr_time();

enum STATUS
{
	INITIALISING = 1,
	READY,
	RECORDING,
	SENSOR_ERROR,
	LOW_BATTERY,
	LOW_MEMORY
};

STATUS rec_status_;

//message variables
sensor_msgs::Imu imu_;			bool newimu_=0;
sensor_msgs::LaserScan lsl_;	bool newlsl_=0;
sensor_msgs::LaserScan lsm_;	bool newlsm_=0;
std_msgs::UInt8 usr_;			bool newusr_=0;
std_msgs::UInt8 io_write_;

//flags
bool start_rec = 0, stop_rec = 0, rec=0, bopen_=0;
bool rec_lsm = 0, rec_cam = 0, rec_disk = 0;

//ros variables
int HZ;
std::string PATH, fname_;
//rosbag::Bag bag;
int prev_cmd = 0;

std::queue<baseMsg*> m_msgqueue;

//process running status
bool m_exitThread=false;

//Object that handles rosbag
Record * m_recordobj;

//mutex to guard the queue
boost::mutex m_guard;

//mutex to guard the recordflag
boost::mutex m_guardrecord;

//status flag vector to indicate the health of each conncted sensor
unsigned short m_sensorstatus=0;
unsigned short ALL_SENSORS=0;

//Timers to manage individual sensor recording frequencies
bool m_device[NUMOFDEVICES];
double m_device_timer[NUMOFDEVICES];
bool m_recordflag;

//publisher topics 
const std::string usr_wtopic_ = "/sb_status";
ros::Publisher pub_usr_;
ros::Publisher pub_dis_;

//subscribe topics
const std::string imu_topic_ = "/myahrs_imu";
ros::Subscriber sub_imu;
ros::Timer imu_timer;
const std::string lsl_topic_ = "/scan_lsl";
ros::Subscriber sub_lsl;
ros::Timer lsl_timer;
const std::string lsm_topic_ = "/scan_lsm";
ros::Subscriber sub_lsm;
ros::Timer lsm_timer;
const std::string cam_topic_ = "/camera/image_raw";
ros::Subscriber sub_cam;
ros::Timer cam_timer;
const std::string dis_topic_ = "/disk_stats";
ros::Subscriber sub_dis;
ros::Timer dis_timer;
const std::string usr_topic_ = "/sb_switches";
ros::Subscriber sub_usr;
const std::string tfs_topic_ = "/tf";
ros::Subscriber sub_tfs;
