#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

/** @file conv_cloud_to_cloud2.cpp
 *  @brief Subscribes to a PointCloud message and publishes a PointCloud2 message.
 *
 *  @author Brendan Emery
 *  @date Jan 2016
 *  @version 1.0.0
 *  @bug Currently no known bugs.
 *  @todo Currently no todos.
 */

void pointCloudCallBack(const sensor_msgs::PointCloud::ConstPtr&);

ros::Publisher point_cloud_pub;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "conv_cloud_to_cloud2");
    ros::NodeHandle n;


    // Subscribers
    ros::Subscriber point_cloud_sub = n.subscribe<sensor_msgs::PointCloud>
                                      ("cloud_in", 100, pointCloudCallBack);

    // Publishers
    point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("cloud_to_cloud2_out", 100);

    ros::spin();
}


/*
    Callback to convert the PointCloud message to PointCloud2 and publish the PointCloud2 message.
*/
void pointCloudCallBack(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    sensor_msgs::PointCloud2 pointCloud2Out;
    sensor_msgs::convertPointCloudToPointCloud2(*msg, pointCloud2Out);

    std::cout << "Converted PointCloud to PointCloud2" << std::endl;
    point_cloud_pub.publish(pointCloud2Out);
}
