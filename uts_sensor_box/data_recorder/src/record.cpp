/*
 * utsrecord.cpp
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

#include "record.h"

#include <csignal>
#include <iostream>
#include <sstream>

using namespace Sensor_Box_pkg;
using namespace std;
using namespace boost;
using namespace boost::this_thread;

//Record
Record::Record(ros::NodeHandle* nh_) : m_exitThread(false)
{
    mp_nh = nh_;

    mp_nh->param<string> ( "file_path", file_path, "/home/odroid/ros_bag/" );

    //Open a rosbag to share between nodes
    //Assign unique name to bag file
    //stringstream temp_stream, temp_stream2,temp_stream3;
    //string bagname;
    //double File_time = ros::Time::now().toSec();

    //temp_stream2 <<file_path<< "sb_data_" <<fixed << File_time << ".bag";

    //bagname = temp_stream2.str();

    //m_bag.open(bagname.c_str(), rosbag::bagmode::Write);
    //m_bag.setCompression(rosbag::CompressionType(1));

    //ROS_INFO("%s\n", bagname.c_str());

	//newBag();

    //Create the writing thread
    boost::thread thread1(boost::bind(&Record::operator(),this));
}

Record::~Record()
{
    while(!m_recordqueue.empty())
    {
        write();
    }
    m_bag.close();
    //m_bag_compressed.close();

    ROS_INFO( "recording module detached");
}

void Record::newBag()
{
    //Open a rosbag to share between nodes
    //Assign unique name to bag file
    stringstream temp_stream, temp_stream2,temp_stream3;
    string bagname;
    double File_time = ros::Time::now().toSec();

    temp_stream2 <<file_path<< "sb_data_" <<fixed << File_time << ".bag";

    bagname = temp_stream2.str();

    m_bag.open(bagname.c_str(), rosbag::bagmode::Write);
    //m_bag.setCompression(rosbag::CompressionType(1));

    ROS_INFO("%s\n", bagname.c_str());
}

void Record::closeBag()
{
    while(!m_recordqueue.empty())
    {
        write();
    }
    m_bag.close();
}

void Record::exitIndication(int sig)
{
    m_guard.lock();
    m_exitThread = (sig==SIGINT);
    m_guard.unlock();
}

void Record::record(baseMsg* msg_)
{
    m_guard.lock();
    m_recordqueue.push(msg_);
    m_guard.unlock();
}

void Record::operator() ()
{
    ROS_INFO( "[record] Objected created and operating");

    ros::Rate r(200);
    bool _threadexit = false;

    m_guard.lock();
    _threadexit = m_exitThread;
    m_guard.unlock();

    while(!_threadexit)
    {
        write();
        m_guard.lock();
        _threadexit = m_exitThread;
        m_guard.unlock();

        if(!_threadexit)
        {
            ros::spinOnce();
            r.sleep();
        }
    }

    ROS_INFO( "Exiting Record thread");
    return;
}

void Record::write()
{
    baseMsg* msg_ ;

    bool qempty = true;

    static double begin = ros::Time::now().toSec();

    double current_time;

    m_guard.lock();
    if(m_recordqueue.empty())
    {
        m_guard.unlock();
        return;
    }
    else
    {
        qempty = false;
        m_guard.unlock();
    }

    while(!qempty)
    {
        current_time = ros::Time::now().toSec();

        if ((current_time - begin > 10 )&&( m_recordqueue.size() >0))
        {
            ROS_INFO( "[record] queue len= %d" , m_recordqueue.size()  );

            begin =current_time;
        }

        m_guard.lock();
        msg_ = m_recordqueue.front();
        m_recordqueue.pop();
        m_guard.unlock();

        if (*msg_ == baseMsg::IMU)
        {
            //Writing to rosbag
            imuRealMsg* imurealmsg_ = reinterpret_cast<imuRealMsg*>(msg_);
            sensor_msgs::ImuConstPtr real_imuIn = imurealmsg_->getMessage();

            m_bag.write(imurealmsg_->m_topic.c_str(), real_imuIn->header.stamp, real_imuIn);
            delete imurealmsg_;
        }        
        else if (*msg_ == baseMsg::LASER_LSL)
        {
            //Writing to rosbag
            laserLSLMsg* lasermsg_ = reinterpret_cast<laserLSLMsg*>(msg_);
            sensor_msgs::LaserScanConstPtr laserIn = lasermsg_->getMessage();

            m_bag.write(lasermsg_->m_topic.c_str(), laserIn -> header.stamp, laserIn);
            delete lasermsg_;
        }
        else if (*msg_ == baseMsg::LASER_LSM)
        {
            //Writing to rosbag
            laserLSMMsg* lasermsg_ = reinterpret_cast<laserLSMMsg*>(msg_);
            sensor_msgs::LaserScanConstPtr laserIn = lasermsg_->getMessage();

            m_bag.write(lasermsg_->m_topic.c_str(), laserIn -> header.stamp, laserIn);
            delete lasermsg_;
        }
        else if (*msg_ == baseMsg::CAM)
        {
            //Writing to rosbag
            imageMsg* imagemsg_ = reinterpret_cast<imageMsg*>(msg_);
            sensor_msgs::ImageConstPtr imageIn = imagemsg_->getMessage();

            m_bag.write(imagemsg_->m_topic.c_str(), imageIn -> header.stamp, imageIn);
            delete imagemsg_;
        }
		else if(*msg_ == baseMsg::TF)
		{
			tfMsg* tfMsg_ = reinterpret_cast<tfMsg*>(msg_);
			tf::tfMessageConstPtr tfMsgIn = tfMsg_->getMessage();

			m_bag.write(tfMsg_->m_topic.c_str(), tfMsgIn->transforms[0].header.stamp, tfMsgIn);
			delete tfMsg_;
		}
        else if (*msg_ == baseMsg::DISK)
        {
            //Writing to rosbag
            fsMntrMsg* diskmsg_ = reinterpret_cast<fsMntrMsg*>(msg_);
            data_recorder::Fs_spaceConstPtr diskIn = diskmsg_->getMessage();

            m_bag.write(diskmsg_->m_topic.c_str(), diskIn -> header.stamp, diskIn);
            delete diskmsg_;
        }

        m_guard.lock();
        if(m_recordqueue.empty())
        {
            qempty = true;
            m_guard.unlock();

        }
        else
        {
            qempty = false;
            m_guard.unlock();
        }

    }
}
