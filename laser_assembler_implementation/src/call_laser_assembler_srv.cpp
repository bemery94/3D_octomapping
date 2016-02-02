#include <ros/ros.h>
#include <laser_assembler/AssembleScans.h>
#include <sensor_msgs/PointCloud.h>

using namespace laser_assembler;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "call_laser_assembler_srv");
    ros::NodeHandle n;
    ros::service::waitForService("assemble_scans");
    ros::ServiceClient client = n.serviceClient<AssembleScans>("assemble_scans");

    ros::Publisher my_pub = n.advertise<sensor_msgs::PointCloud>("point_cloud_publisher", 10);

    AssembleScans srv;
    ros::Rate loop_rate(10);
    while(ros::ok()) {


        srv.request.begin = ros::Time(0, 0);
        srv.request.end = ros::Time::now();
        sensor_msgs::PointCloud my_cloud;
        if (client.call(srv)) {
            printf("Got cloud with %u points");
            my_cloud = srv.response.cloud;
        }
        else
            printf("Service call failed\n");
        my_pub.publish(my_cloud);
        std::cout << "Sucess" << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }



    return 0;
}
