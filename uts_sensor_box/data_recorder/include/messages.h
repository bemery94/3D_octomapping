/*
 * utsmessages.h
 *
 * Copyright (c) 2013, Centre for Intelligent Mechatronics Systems, University of Technology, Sydney, Australia.
 * All rights reserved.
 *
 * This software was developed as a part of an industry based research project on Assistive Robotic Devices.
 *
 * Author: Ravindra Ranasinghe
 * Date: 04/04/2013
 *
 * Rev History:
 *       0.0 - Ravindra Ranasinghe
 *       0.1 - Lakshitha Dantanarayana
 *       0.2 - Dave Hunt - modified for sensor box - P.O.W project
 */

#pragma once

///ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <tf/tfMessage.h>


#include <data_recorder/Fs_space.h>

//Namespace for sensor box
namespace Sensor_Box_pkg
{
//baseMsg class
class baseMsg
{
public:
	typedef enum MsgType__ {CAM, IMU, LASER_LSL, LASER_LSM, DISK, TF} MsgType;

	//Constructor
	baseMsg(MsgType type_, std::string topic_);

        //Copy constructor
        baseMsg(const baseMsg & bmsg_);

        //Destructor
        virtual ~baseMsg();

        //Accessors
	bool operator==(MsgType type_);

public:
	MsgType	m_type;
	std::string	m_topic;

private:
        static unsigned int ms_objcnt;
};

//imuRealMessage class
class imuRealMsg : public baseMsg
{
public:
        //Constructor
        imuRealMsg(std::string topic_);

        //Copy constructor
        imuRealMsg(const imuRealMsg & msg);

        //Destructor
        virtual ~imuRealMsg();


        //Modifiers
        void setMessage(const sensor_msgs::ImuConstPtr&);

        //Accessors
        sensor_msgs::ImuConstPtr getMessage(void);
private:
        sensor_msgs::ImuConstPtr mp_msgIn;

};

//LaserMessage class
class laserLSLMsg : public baseMsg
{
public:
        //Constructor
        laserLSLMsg(std::string topic_);

        //Copy constructor
        laserLSLMsg(const laserLSLMsg & msg);

        //Destructor
        virtual ~laserLSLMsg();


        //Modifiers
        void setMessage(const sensor_msgs::LaserScanConstPtr&);

        //Accessors
        sensor_msgs::LaserScanConstPtr getMessage(void);
private:
        sensor_msgs::LaserScanConstPtr mp_msgIn;

};

//LaserMessage class
class laserLSMMsg : public baseMsg
{
public:
        //Constructor
        laserLSMMsg(std::string topic_);

        //Copy constructor
        laserLSMMsg(const laserLSMMsg & msg);

        //Destructor
        virtual ~laserLSMMsg();


        //Modifiers
        void setMessage(const sensor_msgs::LaserScanConstPtr&);

        //Accessors
        sensor_msgs::LaserScanConstPtr getMessage(void);
private:
        sensor_msgs::LaserScanConstPtr mp_msgIn;

};

//image class
class imageMsg : public baseMsg
{
public:
        //Constructor
        imageMsg(std::string topic_);

        //Copy constructor
        imageMsg(const imageMsg & msg);

        //Destructor
        virtual ~imageMsg();

        //Modifiers
        void setMessage(const sensor_msgs::ImageConstPtr&);

        //Accessors
        sensor_msgs::ImageConstPtr getMessage(void);
private:
        sensor_msgs::ImageConstPtr mp_msgIn;
};

//tf message class
class tfMsg : public baseMsg
{
public:
        //Constructor
        tfMsg(std::string topic_);

        //Copy constructor
        tfMsg(const tfMsg & msg);

        //Destructor
        virtual ~tfMsg();


        //Modifiers
        void setMessage(const tf::tfMessageConstPtr&);

        //Accessors
        tf::tfMessageConstPtr getMessage(void);
private:
        tf::tfMessageConstPtr mp_msgIn;

};

class fsMntrMsg : public baseMsg
{
public:
        //Constructor
        fsMntrMsg(std::string topic_);

        //Copy constructor
        fsMntrMsg(const fsMntrMsg & msg);

        //Destructor
        virtual ~fsMntrMsg();

        //Modifiers
        void setMessage(const data_recorder::Fs_spaceConstPtr&);

        //Accessors
        data_recorder::Fs_spaceConstPtr getMessage(void);
private:
        data_recorder::Fs_spaceConstPtr mp_msgIn;

};

}
