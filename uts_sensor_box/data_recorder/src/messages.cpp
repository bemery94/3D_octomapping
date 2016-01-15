/*
 * utsmessages.cpp
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

#include "messages.h"

using namespace Sensor_Box_pkg;
using namespace std;

using namespace sensor_msgs;

//baseMsg class

unsigned int baseMsg::ms_objcnt = 0;

baseMsg::baseMsg(MsgType type_, std::string topic_) : m_type(type_), m_topic(topic_)
{
    ++ms_objcnt;
}

baseMsg::baseMsg(const baseMsg & bmsg_)
{
    ++ms_objcnt;
    m_type = bmsg_.m_type;
    m_topic = bmsg_.m_topic;
}

baseMsg::~baseMsg()
{
    --ms_objcnt;
    //std::cout << ms_objcnt << " no. of baseMsg in the memory\n";
}

bool baseMsg::operator==(MsgType type_)
{
    return (m_type==type_);
}

//imuRealMessage class - Placeholder for Real IMU message
imuRealMsg::imuRealMsg(std::string topic_):baseMsg(IMU,topic_)
{
}

imuRealMsg::imuRealMsg(const imuRealMsg & msg_) : baseMsg(msg_)
{
    mp_msgIn = msg_.mp_msgIn;
}

imuRealMsg::~imuRealMsg()
{

}

void imuRealMsg::setMessage(const sensor_msgs::ImuConstPtr& msg_)
{
    mp_msgIn = msg_;
}

sensor_msgs::ImuConstPtr imuRealMsg::getMessage(void)
{
    return mp_msgIn;
}

//LaserScan LSL Message class - Placeholder for LaserScan message
laserLSLMsg::laserLSLMsg(std::string topic_):baseMsg(LASER_LSL,topic_)
{
}

laserLSLMsg::laserLSLMsg(const laserLSLMsg & msg_) : baseMsg(msg_)
{
    mp_msgIn = msg_.mp_msgIn;
}

laserLSLMsg::~laserLSLMsg()
{

}

void laserLSLMsg::setMessage(const sensor_msgs::LaserScanConstPtr& msg_)
{
    mp_msgIn = msg_;
}

sensor_msgs::LaserScanConstPtr laserLSLMsg::getMessage(void)
{
    return mp_msgIn;
}


//LaserScan LSM Message class - Placeholder for LaserScan message
laserLSMMsg::laserLSMMsg(std::string topic_):baseMsg(LASER_LSM,topic_)
{
}

laserLSMMsg::laserLSMMsg(const laserLSMMsg & msg_) : baseMsg(msg_)
{
    mp_msgIn = msg_.mp_msgIn;
}

laserLSMMsg::~laserLSMMsg()
{

}

void laserLSMMsg::setMessage(const sensor_msgs::LaserScanConstPtr& msg_)
{
    mp_msgIn = msg_;
}

sensor_msgs::LaserScanConstPtr laserLSMMsg::getMessage(void)
{
    return mp_msgIn;
}


//imgMsg class - Placeholder for RGB image message
imageMsg::imageMsg(std::string topic_):baseMsg(CAM,topic_)
{
}

imageMsg::imageMsg(const imageMsg & msg_) : baseMsg(msg_)
{
    mp_msgIn = msg_.mp_msgIn;
}

imageMsg::~imageMsg()
{

}

void imageMsg::setMessage(const sensor_msgs::ImageConstPtr & msg_)
{
    mp_msgIn = msg_;
}

sensor_msgs::ImageConstPtr imageMsg::getMessage(void)
{
    return mp_msgIn;
}

//TF  Message class - Placeholder for tf message
tfMsg::tfMsg(std::string topic_):baseMsg(TF,topic_)
{
}

tfMsg::tfMsg(const tfMsg & msg_) : baseMsg(msg_)
{
    mp_msgIn = msg_.mp_msgIn;
}

tfMsg::~tfMsg()
{

}

void tfMsg::setMessage(const tf::tfMessageConstPtr& msg_)
{
    mp_msgIn = msg_;
}

tf::tfMessageConstPtr tfMsg::getMessage(void)
{
    return mp_msgIn;
}

//disk check messages (space left on disk)
fsMntrMsg::fsMntrMsg(std::string topic_):baseMsg(DISK,topic_)
{

}

fsMntrMsg::fsMntrMsg(const fsMntrMsg & msg_) : baseMsg(msg_)
{
    mp_msgIn = msg_.mp_msgIn;
}

fsMntrMsg::~fsMntrMsg()
{

}

void fsMntrMsg::setMessage(const data_recorder::Fs_spaceConstPtr& msg_)
{
    mp_msgIn = msg_;
}

data_recorder::Fs_spaceConstPtr fsMntrMsg::getMessage(void)
{
    return mp_msgIn;
}
