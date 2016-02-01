#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
//#include <tf/tf.h>
#include <tf/transform_listener.h>


/** @file conv_laser_to_cloud.cpp
 *  @brief Subscribes to a LaserScan message and publishes a PointCloud message.
 *
    *
 *  @author Brendan Emery
 *  @date Jan 2016
 *  @version 1.0.0
 *  @bug Currently no known bugs.
 *  @todo Currently no todos.
 */

void laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr&);

ros::Publisher point_cloud_pub;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "conv_laser_to_cloud");
    ros::NodeHandle n;

    ros::Duration test;
    test.sec = 100.0;
    tf::TransformListener listener_(test);

    // Subscribers
    ros::Subscriber laser_scan_sub = n.subscribe<sensor_msgs::LaserScan>
                                      ("laser_scan_in", 100, laserScanCallBack);

//     Publishers
    point_cloud_pub = n.advertise<sensor_msgs::PointCloud>("point_cloud_out", 100);

    ros::spin();
}


/*
    Callback to convert the LaserScan message to PointCloud and publish the PointCloud message.
*/
void laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    sensor_msgs::PointCloud cloud;
    laser_geometry::LaserProjection projector_;

//    while(!listener_.canTransform(scan_in->header.frame_id, "/map_world_frame", ros::Time(0)) && ros::ok())
//        {
//        ros::Duration(1).sleep();
//        if(listener_.canTransform(scan_in->header.frame_id, "/map_world_frame", ros::Time(0)))
//        break;
//        ros::Duration(1).sleep();
//        if(listener_.canTransform(scan_in->header.frame_id, "/map_world_frame", ros::Time(0)))
//        break;
//        ros::Duration(1).sleep();
//        if(listener_.canTransform(scan_in->header.frame_id, "/map_world_frame", ros::Time(0)))
//        break;
//        ros::Duration(1).sleep();
//        ROS_INFO("Cannot retrieve thruster TF's! Ensure that the TF publisher\
//                  node is running");
//        }

    ROS_INFO_STREAM(ros::Time::now());
    ROS_INFO_STREAM(scan_in->header.stamp);
    ROS_INFO_STREAM(ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment));
    std::cout << "outside while loop" << std::endl;
    if(!listener_.waitForTransform(
        "/base_link",
        //scan_in->header.frame_id,
        "/laser_lsl",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment) + ros::Duration(0.3),
        ros::Duration(5.0)))
        {
            ROS_INFO_STREAM(ros::Time::now());
            std::cout << "Exiting" << std::endl;
            return;
        }
//        projector_.transformLaserScanToPointCloud ("/base_link", *scan_in, cloud, listener_);

//    ROS_INFO_STREAM(listener_.frameExists("/map_world_frame"));

    std::cout << "After" << std::endl;



    std::cout << "Converted LaserScan to PointCloud" << std::endl;
    point_cloud_pub.publish(cloud);

}
