#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include <math.h>
#include <std_msgs/Float32MultiArray.h>
#include "Eigen/Dense"

//ros::Publisher my_pub1;
//ros::Publisher my_pub2;
ros::Publisher orientation_pub;

Eigen::Matrix<double,3,3> convQuatToRot(double, double, double, double);

void callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Declare Orientation msg to store roll and pitch
    sensor_msgs::JointState orientation_val;

    // Transfer time stamp from quaternion msg to Orientation msg
    orientation_val.header.stamp = msg->header.stamp;

    // Convert quaternion to roll and pitch values
        float x;
    float y;
    float z;
    float w;

    float roll;
    float pitch;
    float yaw;

    float roll1;
    float pitch1;
    float yaw1;

    float roll2;
    float pitch2;
    float yaw2;

    x = msg->orientation.x;
    y = msg->orientation.y;
    z = msg->orientation.z;
    w = msg->orientation.w;

    Eigen::Matrix<double,3,3> myRotMat;
    myRotMat = convQuatToRot(x, y, z, w);

    pitch = atan2(-myRotMat(2,0), sqrt(myRotMat(0,0) * myRotMat(0,0) + myRotMat(1,0) * myRotMat(1,0)));
    yaw = atan2(myRotMat(1,0) / cos(pitch), myRotMat(0,0) / cos(pitch));
    roll = atan2(myRotMat(2,1) / cos(pitch), myRotMat(2,2) / cos(pitch));

//    std_msgs::Float32MultiArray array_rot_mat;
//
//    array_rot_mat.data.resize(3);
//    array_rot_mat.data[0] = roll * 180 / 3.14159265359;
//    array_rot_mat.data[1] = pitch * 180 / 3.14159265359;
//    array_rot_mat.data[2] = yaw * 180 / 3.14159265359;

//    my_pub_rot_mat.publish(array_rot_mat);

//    roll1 = atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z);
//    pitch1 = atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z);
//    yaw1 =  asin(2*x*y + 2*z*w);
//
//    yaw2   =  atan2(2*x*y + 2*w*z, w*w + x*x - y*y - z*z);
//    pitch2 = -asin(2*w*y - 2*x*z);
//    roll2  = -atan2(2*y*z + 2*w*x, -w*w + x*x + y*y - z*z)  ;
//
//
//    std_msgs::Float32MultiArray array_1;
//    std_msgs::Float32MultiArray array_2;
//
//    array_1.data.resize(3);
//    array_1.data[0] = roll1 * 180 / 3.14159265359;
//    array_1.data[1] = pitch1 * 180 / 3.14159265359;
//    array_1.data[2] = yaw1 * 180 / 3.14159265359;
//
//    array_2.data.resize(3);
//    array_2.data[0] = roll2 * 180 / 3.14159265359;
//    array_2.data[1] = pitch2 * 180 / 3.14159265359;
//    array_2.data[2] = yaw2 * 180 / 3.14159265359;
//
//    my_pub1.publish(array_1);
//    my_pub2.publish(array_2);

    // Store roll and pitch values in Orientation msg
    orientation_val.roll = roll;
    orientation_val.pitch = pitch;
    orientation_val.yaw = yaw;

    orientation_pub.publish(orientation_val);
}

Eigen::Matrix<double,3,3> convQuatToRot(double x, double y, double z, double w)
{
    Eigen::Matrix<double,3,3> rotMatOut;

    rotMatOut(0,0) = 1 - 2 * pow(y, 2) - 2
                  * pow(z, 2);
    rotMatOut(0,1) = 2 * x * y
                  - 2 * z * w;
    rotMatOut(0,2) = 2 * x * z + 2
                  * y * w;
    rotMatOut(1,0) = 2 * x * y + 2
                  * z * w;
    rotMatOut(1,1) = 1 - 2 * pow(x, 2) - 2
                  * pow(z, 2);
    rotMatOut(1,2) = 2 * y * z
                  - 2 * x * w;
    rotMatOut(2,0) = 2 * x * z
                  - 2 * y * w;
    rotMatOut(2,1) = 2 * y * z
                  + 2 * x * w;
    rotMatOut(2,2) = 1 - 2 * pow(x, 2) - 2
                  * pow(y, 2);

    return rotMatOut;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "orientation_state_publisher");
    ros::NodeHandle n;
    ros::Subscriber my_sub = n.subscribe<sensor_msgs::Imu>("myahrs_imu", 100, callback);
    orientation_pub = n.advertise<data_recorder::Orientation("scanner_orientation", 100);
//    my_pub1 = n.advertise<std_msgs::Float32MultiArray>("roll_and_pitch_out1", 100);
//    my_pub2 = n.advertise<std_msgs::Float32MultiArray>("roll_and_pitch_out2", 100);
//    my_pub_rot_mat = n.advertise<std_msgs::Float32MultiArray>("roll_and_pitch_out_rot_mat", 100);

    ros::spin();
}

