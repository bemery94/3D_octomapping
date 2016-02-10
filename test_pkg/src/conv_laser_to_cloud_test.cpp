#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>

/** @file conv_laser_to_cloud.cpp
 *  @brief Subscribes to a LaserScan message and publishes a PointCloud message.
 *
 *  @author Brendan Emery
 *  @date Jan 2016
 *  @version 1.0.0
 *  @bug Currently no known bugs.
 *  @todo Currently no todos.
 */

void laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr&);

ros::Publisher point_cloud_pub;
laser_geometry::LaserProjection projector_;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "conv_laser_to_cloud_test");
    ros::NodeHandle n;


    // Subscribers
    ros::Subscriber laser_scan_sub = n.subscribe<sensor_msgs::LaserScan>
                                      ("laser_scan_in", 100, laserScanCallBack);

    // Publishers
    point_cloud_pub = n.advertise<sensor_msgs::PointCloud>("laser_to_cloud_out", 100);

    ros::spin();
}


/*
    Callback to convert the LaserScan message to PointCloud and publish the PointCloud message.
*/
void laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensor_msgs::PointCloud cloud;
    projector_.projectLaser(*msg, cloud);

    std::cout << "Converted LaserScan to PointCloud" << std::endl;
    point_cloud_pub.publish(cloud);
}
