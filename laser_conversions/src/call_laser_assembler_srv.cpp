//#include <ros/ros.h>
//#include <laser_assembler/AssembleScans.h>
//#include <sensor_msgs/PointCloud.h>
//#include "tf/transform_listener.h"
//#include "pcl_ros/transforms.h"
//#include "../../../../../../../../opt/ros/indigo/include/tf/transform_listener.h"
//
//
//using namespace laser_assembler;
//int main(int argc, char **argv)
//{
//    ros::init(argc, argv, "call_laser_assembler_srv");
//    ros::NodeHandle n;
//    ros::service::waitForService("assemble_scans");
//    ros::ServiceClient client = n.serviceClient<AssembleScans>("assemble_scans");
//
//    tf::TransformListener listener_;
//
//    ros::Publisher my_pub = n.advertise<sensor_msgs::PointCloud>("assembled_cloud_out", 10);
//
//    AssembleScans srv;
//    ros::Rate loop_rate(10);
//    while(ros::ok()) {
////        ros::Duration two_seconds(5.0);
//        srv.request.begin = ros::Time(0,0); //ros::Time::now() - two_seconds;
//        srv.request.end = ros::Time::now();
//        sensor_msgs::PointCloud myCloud;
//        sensor_msgs::PointCloud transformedCloud;
//
//        if (client.call(srv)) {
//            myCloud = srv.response.cloud;
//        }
//        else
//            printf("Service call failed\n");
//
//
//
//        my_pub.publish(myCloud);
//        std::cout << "Sucess" << std::endl;
//
//        ros::spinOnce();
//        loop_rate.sleep();
//    }
//
//
//
//    return 0;
//}

#include <cstdio>
#include <ros/ros.h>
#include "tf/transform_listener.h"
#include "Eigen/Dense"

// Services
#include "laser_assembler/AssembleScans.h"

// Messages
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "../../../../../../../../opt/ros/indigo/include/ros/forwards.h"

#include <pcl/conversions.h>
#include "pcl_ros/transforms.h"


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
/***
 * This a simple test app that requests a point cloud from the
 * point_cloud_assembler every 4 seconds, and then publishes the
 * resulting data
 */
namespace laser_assembler
{

class PeriodicSnapshotter
{

public:

  PeriodicSnapshotter()
  {
    // Create a publisher for the clouds that we assemble
    pub_ = n_.advertise<sensor_msgs::PointCloud> ("assembled_cloud_out", 1);

    // Create the service client for calling the assembler
    client_ = n_.serviceClient<AssembleScans>("assemble_scans");

    // Start the timer that will trigger the processing loop (timerCallback)
    timer_ = n_.createTimer(ros::Duration(1), &PeriodicSnapshotter::timerCallback, this);

    // Need to track if we've called the timerCallback at least once
    first_time_ = true;
  }

  void timerCallback(const ros::TimerEvent& e)
  {

    // We don't want to build a cloud the first callback, since we we
    //   don't have a start and end time yet
    if (first_time_)
    {
      first_time_ = false;
      return;
    }

    // Populate our service request based on our timer callback times
    AssembleScans srv;
    srv.request.begin = e.last_expected;
    srv.request.end   = e.current_expected;

    // Make the service call
    if (client_.call(srv))
    {

        tf::StampedTransform localTransform;

        try
        {
          listener_.lookupTransform("/base_footprint", "/map_world_frame",
                                   ros::Time(0), localTransform);
        }
        catch (tf::TransformException &ex)
        // If the tf listener cannot find the transform, print an error and continue
    {
      ROS_ERROR("In imu_to_base_pub %s",ex.what());
    }
        sensor_msgs::convertPointCloudToPointCloud2(srv.response.cloud, initialCloud2);
//        pcl::fromROSMsg(initialCloud2, pclPointCloud);
        pcl_ros::transformPointCloud("/base_footprint", localTransform, initialCloud2,
                                     transformedCloud2);
        sensor_msgs::convertPointCloud2ToPointCloud(transformedCloud2, transformedCloud);
//        listener_.transformPointCloud("/base_footprint", srv.response.cloud, transformedCloud);
      ROS_INFO("Published Cloud with %u points", (uint32_t)(srv.response.cloud.points.size()));
        transformedCloud.header.frame_id = "base_footprint";
      pub_.publish(transformedCloud);
    }
    else
    {
      ROS_ERROR("Error making service call\n") ;
    }
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::ServiceClient client_;
    sensor_msgs::PointCloud2 transformedCloud2;
    sensor_msgs::PointCloud2 initialCloud2;
    sensor_msgs::PointCloud transformedCloud;
    pcl::PointCloud<pcl::PointXYZ> pclPointCloud;
  ros::Timer timer_;
  bool first_time_;
    tf::TransformListener listener_;
} ;

}

using namespace laser_assembler ;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "periodic_snapshotter");
  ros::NodeHandle n;
  ROS_INFO("Waiting for [build_cloud] to be advertised");
  ros::service::waitForService("build_cloud");
  ROS_INFO("Found build_cloud! Starting the snapshotter");
  PeriodicSnapshotter snapshotter;
  ros::spin();
  return 0;
}
