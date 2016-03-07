#include <pcl/io/pcd_io.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/conversions.h>

pcl::PCLPointCloud2 cloud2Lsl;
pcl::PCLPointCloud2 cloud2Lsm;

pcl::PointCloud<pcl::PointXYZ> cloudLsl;
pcl::PointCloud<pcl::PointXYZ> cloudLsm;

void lsl_cb(const sensor_msgs::PointCloud2::ConstPtr&);
void lsm_cb(const sensor_msgs::PointCloud2::ConstPtr&);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "concatenate_pcls");
	ros::NodeHandle n;

	ros::Subscriber lsl_sub = n.subscribe<sensor_msgs::PointCloud2>("/cloud_to_cloud2_out_lsl", 10,
	                                                                lsl_cb);
	ros::Subscriber lsm_sub = n.subscribe<sensor_msgs::PointCloud2>("/cloud_to_cloud2_out_lsm", 10,
	                                                                lsm_cb);
	ros::Publisher assembed_cloud_out = n.advertise<sensor_msgs::PointCloud2>
			("/assembled_cloud2_out", 10);

	pcl::PointCloud<pcl::PointXYZ> cloudOut;
	pcl::PCLPointCloud2 cloud2Out;
	sensor_msgs::PointCloud2 sensorCloud2Out;

	ros::Rate sleep_rate(10);
	while(ros::ok())
	{
		cloudOut = cloudLsl + cloudLsm;

		pcl::toPCLPointCloud2(cloudOut, cloud2Out);
		pcl_conversions::fromPCL(cloud2Out, sensorCloud2Out);

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

