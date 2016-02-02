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
tf::TransformListener* listener_ = NULL;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "conv_laser_to_cloud");
    ros::NodeHandle n;

    listener_ = new tf::TransformListener(ros::Duration(100));

    // Subscribers
    ros::Subscriber laser_scan_sub = n.subscribe<sensor_msgs::LaserScan>
                                      ("laser_scan_in", 100, laserScanCallBack);

    // Publishers
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


    if(!listener_->waitForTransform(
        scan_in->header.frame_id,
        "/map_world_frame",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(5.0)))
        {
            std::cout << "Cannot find TF between " << scan_in->header.frame_id <<
                            "and map_world_frame" << std::endl;
            return;
        }
        projector_.transformLaserScanToPointCloud ("/map_world_frame", *scan_in, cloud, *listener_);

    std::cout << "Converted LaserScan to PointCloud from " << scan_in->header.frame_id << "to ==> "
              << "map_world_frame" << std::endl;

    point_cloud_pub.publish(cloud);
}
