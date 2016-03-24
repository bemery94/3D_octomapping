#include <pcl/io/pcd_io.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/conversions.h>
#include "tf/transform_listener.h"
#include "pcl/common/transforms.h"
#include "pcl_ros/transforms.h"
#include "../../../../../../../../opt/ros/indigo/include/ros/time.h"
#include <sensor_msgs/point_cloud_conversion.h>

/** @file concatenate_pcls.cpp
 *  @brief Combines the point clouds from the horizontal and vertical laser scanners to produce a
 *  single point cloud.
 *
 *  This node is not required in the current pipeling of the laser scanner as the horizontal
 *  scans in the concatenated point cloud cause issues when assembling the scans (using the
 *  laser_assembler package) before being passed into the octomap.
 *
 *  @author Brendan Emery
 *  @date March 2016
 *  @version 1.0.0
 *  @bug Currently no known bugs.
 *  @todo Currently no todos.
 */

pcl::PCLPointCloud2 cloud2Lsl;
pcl::PCLPointCloud2 cloud2Lsm;

pcl::PointCloud<pcl::PointXYZ> cloudLsl;
pcl::PointCloud<pcl::PointXYZ> cloudLsm;

void lsl_cb(const sensor_msgs::PointCloud2::ConstPtr&);
void lsm_cb(const sensor_msgs::PointCloud2::ConstPtr&);
tf::StampedTransform getTransform(std::string, std::string, ros::Time);
tf::TransformListener* listener_ = NULL;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "concatenate_pcls");
	ros::NodeHandle n;

	listener_ = new tf::TransformListener();

	ros::Subscriber lsl_sub = n.subscribe<sensor_msgs::PointCloud2>("/cloud_to_cloud2_out_lsl", 10,
	                                                                lsl_cb);
	ros::Subscriber lsm_sub = n.subscribe<sensor_msgs::PointCloud2>("/cloud_to_cloud2_out_lsm", 10,
	                                                                lsm_cb);
	ros::Publisher assembed_cloud_out = n.advertise<sensor_msgs::PointCloud2>
			("/conc_assembled_cloud_out", 10);

	pcl::PointCloud<pcl::PointXYZ> cloudOut;
	pcl::PointCloud<pcl::PointXYZ> transformedCloudLsl;
	pcl::PointCloud<pcl::PointXYZ> transformedCloudLsm;
	pcl::PCLPointCloud2 cloud2Out;
	tf::StampedTransform transformLsl;
	tf::StampedTransform transformLsm;
	sensor_msgs::PointCloud2 sensorCloud2Out;
	sensor_msgs::PointCloud sensorCloudOut;

	transformLsm = getTransform("/base_link", "/laser_lsm", ros::Time(0));
	transformLsl = getTransform("/base_link", "/laser_lsl", ros::Time(0));

	sensorCloud2Out.header.frame_id = "/base_link";

	ros::Rate sleep_rate(40);
	while(ros::ok())
	{
		pcl_ros::transformPointCloud(cloudLsl, transformedCloudLsl, transformLsl);
		pcl_ros::transformPointCloud(cloudLsm, transformedCloudLsm, transformLsm);

		cloudOut = transformedCloudLsl + transformedCloudLsm;

		pcl::toPCLPointCloud2(cloudOut, cloud2Out);
		pcl_conversions::fromPCL(cloud2Out, sensorCloud2Out);

//		sensor_msgs::convertPointCloud2ToPointCloud(sensorCloud2Out, sensorCloudOut);
		assembed_cloud_out.publish(sensorCloud2Out);

		sleep_rate.sleep();
		ros::spinOnce();
	}
}

void lsl_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	// Convert the sensor msg into a PCLPointCloud2 msg
	pcl_conversions::toPCL(*msg, cloud2Lsl);

	// Convert the PCLPointCloud2 msg into a PCLPointCloud msg
	pcl::fromPCLPointCloud2(cloud2Lsl, cloudLsl);
}

void lsm_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	// Convert the sensor msg into a PCLPointCloud2 msg
	pcl_conversions::toPCL(*msg, cloud2Lsm);

	// Convert the PCLPointCloud2 msg into a PCLPointCloud msg
	pcl::fromPCLPointCloud2(cloud2Lsm, cloudLsm);
}

tf::StampedTransform getTransform(const std::string target_frame, const std::string source_frame,
                                  const ros::Time timeStamp)
// Retrieve the transform between the target_frame and source_frame arguments
{
	tf::StampedTransform transformOut;
	while(!listener_->canTransform(target_frame, source_frame, ros::Time(0)) && ros::ok())
    {
        ros::Duration(1).sleep();
        if(listener_->canTransform(target_frame, source_frame, ros::Time(0)))
        break;
        ros::Duration(1).sleep();
        if(listener_->canTransform(target_frame, source_frame, ros::Time(0)))
        break;
        ros::Duration(1).sleep();
        if(listener_->canTransform(target_frame, source_frame, ros::Time(0)))
        break;
        ros::Duration(1).sleep();
        ROS_ERROR("Cannot retrieve TF's. Ensure that scanning data is being published.");
    }

    try
    {
	    listener_->waitForTransform(target_frame, source_frame, timeStamp,
	                               ros::Duration(3.0));
        listener_->lookupTransform(target_frame, source_frame,
                               timeStamp, transformOut);
    }
    catch (tf::TransformException &ex)
    // If the tf listener cannot find the transform, print an error and continue
    {
      ROS_ERROR("In concatenate_pcls %s",ex.what());
    }

	return transformOut;
}
