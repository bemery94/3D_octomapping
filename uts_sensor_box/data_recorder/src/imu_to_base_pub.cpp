#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"
#include "Eigen/Dense"

/** @file imu_to_base_pub.cpp
 *  @brief Transforms the roll and pitch values from an imu message to give the transform between
 *         the base_link and base_stabilized frames.
 *
 *  The notation for rotation matrix used in this node is: rotation giving frame b with respect to
 *  frame a is given by R(a, b).
 *
 *  This node subscribes to the IMU and converts the provided quaternion to a rotation matrix,
 *  giving R(inertial, IMU). It also uses a transform listener to get the TF between the IMU and
 *  the base_link, giving R(base_link, IMU) and the TF between the inertial frame and the map
 *  giving R(inertial, map).
 *
 *  For more information, see coordinate_frame_info.tex in the documentation folder in the root of
 *  the package.
 *
 *  @author Brendan Emery
 *  @date Feb 2016
 *  @version 1.0.0
 *  @bug Currently no known bugs.
 *  @todo Currently no todos.
 */

 // Functional protoypes
tf::Matrix3x3 getRotationMat(std::string, std::string);
Eigen::Matrix<double,3,3> convTransformToRot(tf::StampedTransform);
void imu_cb(const sensor_msgs::Imu::ConstPtr&);

// Global variables
tf::TransformListener* listener_ = NULL;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_to_base_pub");
	ros::NodeHandle n;

	listener_ = new tf::TransformListener();

    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("myahrs_imu", 100, imu_cb);

    ros::spin();
}


void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    /* This callback function gets the roll and pitch values from the IMU, the static transform
       between the IMU and the base link and the static transform between the inertial frame and
       the map frame. It then outputs a transform between the base_link and the base_stabilized
       frame.

       Naming convention for variables: base_link = Bl
                                        base_stabilized = Bs
       */

    tf::Quaternion msgQuat;

    /* Convert the quaternion from the imu message into a TF quaternion so that we can perform
       linear algebra and conversions with the TF library
       */
    tf::quaternionMsgToTF(msg->orientation, msgQuat);

    // Convert the quaternion to a rotation matrix
    tf::Matrix3x3 rotInertialToImu(msgQuat);

    // Get the transform from the imu to the base_link
    tf::Matrix3x3 rotBlToImu;
    rotBlToImu = getRotationMat("/base_link", "/corrected_imu");

    // Calculate the rotation matrix giving the base_link relative to the Inertial frame
    tf::Matrix3x3 rotInertialToBl;
    rotInertialToBl = rotInertialToImu * rotBlToImu.transpose();

    /* Extract the roll and pitch values from the rotation matrix, giving the roll and pitch of the
       base link relative to the stabilized base frame. When converted back into a rotation matrix,
       this will give R(base_stabilized, base_link)
       */
    double roll;
    double pitch;
    double yaw;

    rotInertialToBl.getRPY(roll, pitch, yaw);

    // Calculate the rotation matrix giving the base_stabilized relative to the Inertial frame.
    tf::Matrix3x3 rotInertialToBs;
    rotInertialToBs.setEulerYPR(yaw, 0, 0);
    rotInertialToBs *= rotBlToImu.transpose();

    tf::Matrix3x3 rotBsToBl;
    rotBsToBl = rotInertialToBs.transpose() * rotInertialToBl;

    // Since there is no yaw between base stabilized and base link, we only set the roll and pitch
    tf::Quaternion quatBsToBl;
    rotBsToBl.getRotation(quatBsToBl);

    // Convert the quaternion to a stamped transform
    tf::Transform transformBsToBl;
    transformBsToBl.setRotation(quatBsToBl);

    /* Set the translation between origins to be 0 since the origins of base link and base
       stabilized are at the same point.
       */
    transformBsToBl.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

    tf::StampedTransform transformOut(transformBsToBl, msg->header.stamp, "/base_stabilized",
                                      "/base_link");

    // Setup a TransformBroadcaster and broadcast the TF between base stabilized and base link
    static tf::TransformBroadcaster br;
    br.sendTransform(transformOut);
}


tf::Matrix3x3 getRotationMat(const std::string target_frame, const std::string source_frame)
// Retrieve the transform between the target_frame and source_frame arguments
{
    tf::StampedTransform localTransform;

    try
    {
      listener_->lookupTransform(target_frame, source_frame,
                               ros::Time(0), localTransform);
    }
    catch (tf::TransformException &ex)
    // If the tf listener cannot find the transform, print an error and continue
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // Extract a quaternion from the transform
    tf::Quaternion localQuaternion;
    localQuaternion = localTransform.getRotation();

    // Convert quaternion to rotation matrix
    tf::Matrix3x3 rotMatOut(localQuaternion);

    return rotMatOut;
}


Eigen::Matrix<double,3,3> convTransformToRot(tf::StampedTransform transformIn)
// Converts a StampedTransform into a rotation matrix
{
    Eigen::Matrix<double,3,3> rotMatOut;

    rotMatOut(0,0) = 1 - 2 * pow(transformIn.getRotation().y(), 2) - 2
                  * pow(transformIn.getRotation().z(), 2);
    rotMatOut(0,1) = 2 * transformIn.getRotation().x() * transformIn.getRotation().y()
                  - 2 * transformIn.getRotation().z() * transformIn.getRotation().w();
    rotMatOut(0,2) = 2 * transformIn.getRotation().x() * transformIn.getRotation().z() + 2
                  * transformIn.getRotation().y() * transformIn.getRotation().w();
    rotMatOut(1,0) = 2 * transformIn.getRotation().x() * transformIn.getRotation().y() + 2
                  * transformIn.getRotation().z() * transformIn.getRotation().w();
    rotMatOut(1,1) = 1 - 2 * pow(transformIn.getRotation().x(), 2) - 2
                  * pow(transformIn.getRotation().z(), 2);
    rotMatOut(1,2) = 2 * transformIn.getRotation().y() * transformIn.getRotation().z()
                  - 2 * transformIn.getRotation().x() * transformIn.getRotation().w();
    rotMatOut(2,0) = 2 * transformIn.getRotation().x() * transformIn.getRotation().z()
                  - 2 * transformIn.getRotation().y() * transformIn.getRotation().w();
    rotMatOut(2,1) = 2 * transformIn.getRotation().y() * transformIn.getRotation().z()
                  + 2 * transformIn.getRotation().x() * transformIn.getRotation().w();
    rotMatOut(2,2) = 1 - 2 * pow(transformIn.getRotation().x(), 2) - 2
                  * pow(transformIn.getRotation().y(), 2);

    return rotMatOut;
}



