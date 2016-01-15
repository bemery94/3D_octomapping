/*
 * utsrecord.h
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
 */

#pragma once

///ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/stream.h>

#include <boost/thread.hpp>
#include <queue>

#include "messages.h"

//IRT project namespace
namespace Sensor_Box_pkg
{

class Record
{
public:
    //Constructor
    Record(ros::NodeHandle*  nh_);

    //Destructor
    virtual ~Record();

    //Modifiers
    void exitIndication(int sig);

	void newBag();
	void closeBag();

    //Thread callback
    void operator() ();

    //Record request
    void record(baseMsg* msg_);

    std::string file_path;

private:

    //Methods
    void write();

    //Members
    ros::NodeHandle *mp_nh;
    rosbag::Bag m_bag;
    //rosbag::Bag m_bag_compressed;
    
    bool m_exitThread;
    std::queue<baseMsg*> m_recordqueue;
    boost::mutex m_guard;
};

}
