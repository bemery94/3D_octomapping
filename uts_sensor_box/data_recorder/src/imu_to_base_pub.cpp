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
 *  frame a is given by rotAToB.
 *
 *  This node subscribes to the IMU and converts the provided quaternion to a rotation matrix,
 *  giving rotInertialToImu. It also uses a transform listener to get the TF between the IMU and
 *  the base_link, giving rotBlToImu and the TF between the inertial frame and the map
 *  giving rotInertialToMap.
 *
 *  It uses these transforms to calculate and broadcast the transform between the base_link and
 *  base_stabilized frames.
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
void imu_cb(const sensor_msgs::Imu::ConstPtr&);
tf::Matrix3x3 extractRollPitch(tf::Matrix3x3);

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

	// Declare all rotation matrices
	tf::Matrix3x3 rotInertialToImu(msgQuat);
	tf::Matrix3x3 rotBlToCorrImu;
	tf::Matrix3x3 rotInertialToBl;
	tf::Matrix3x3 rotInertialToMap;
	tf::Matrix3x3 rotMapToBl;
	tf::Matrix3x3 rotMapToBs;
	tf::Matrix3x3 rotInertialToBs;
	tf::Matrix3x3 rotBsToBlRollPitch;
	tf::Matrix3x3 rotBsToBl;
	tf::Matrix3x3 rotCorrImuToImu;
	tf::Matrix3x3 rotInertialToCorrImu;

	// Get rotation matrices from other tf publishers
    rotBlToCorrImu = getRotationMat("/corrected_imu", "/base_link");
	rotCorrImuToImu = getRotationMat("/imu", "/corrected_imu");
	rotInertialToMap = getRotationMat("/map_world_frame", "/inertial");

	rotInertialToCorrImu = rotInertialToImu * rotCorrImuToImu;//.transpose();
	rotInertialToBl = rotInertialToCorrImu * rotBlToCorrImu.transpose();
	rotMapToBl = rotInertialToBl.transpose() * rotInertialToMap;

    double roll;
    double pitch;
    double yaw;
    rotMapToBl.getRPY(roll, pitch, yaw);

	// The base link frame and base stabilized frames are defined as having the same yaw values
	// relative to the map
	rotMapToBs.setEulerYPR(yaw, 0, 0);

	rotInertialToBs = rotInertialToMap * rotMapToBs;
    rotBsToBl = rotInertialToBs.transpose() * rotInertialToBl;

	// We only want to represent the robots roll and pitch, so we remove the yaw value from the
	// rotation of base stabilized ==> base link
	double rollOut;
	double pitchOut;
	double yawOut;

//	rotBsToBlRollPitch = extractRollPitch(rotBsToBl);
	rotBsToBl.getRPY(rollOut, pitchOut, yawOut);
	rotBsToBlRollPitch.setEulerYPR(0, pitchOut, rollOut);

    // Since there is no yaw between base stabilized and base link, we only set the roll and pitch
    tf::Quaternion quatBsToBl;
    rotBsToBlRollPitch.getRotation(quatBsToBl);

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
      ROS_ERROR("In imu_to_base_pub %s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // Extract a quaternion from the transform
    tf::Quaternion localQuaternion;
    localQuaternion = localTransform.getRotation();

    // Convert quaternion to rotation matrix
    tf::Matrix3x3 rotMatOut(localQuaternion);

    return rotMatOut;
}


tf::Matrix3x3 extractRollPitch(const tf::Matrix3x3 rotMatIn)
/* Extracts the roll an pitch values from a rotation matrix and creates a new rotation matrix. It
 * does this by projecting the x and y axes of the original rotation matrix along the xz and yz
 * planes respectively and finding this new rotation.
 * */
{
	tf::Vector3 xVec(1, 0, 0);
	tf::Vector3 yVec(0, 1, 0);

	tf::Vector3 xIn;
	tf::Vector3 yIn;

	// Find the vectors along the x and y axes of the input rotation matrix
	xIn = rotMatIn * xVec;
	yIn = rotMatIn * yVec;

	tf::Vector3 xOut;
	tf::Vector3 yOut;
	tf::Vector3 zOut;

	// Find the x and y vectors projected along the xz and yz frames of the frame that rotMatIn
	// is relative to. We do this by subtracting the component of the xIn and yIn vectors that is
	// normal to the plane that we are projecting them on.
	xOut = xIn - xIn.dot(yVec) * yVec;
	yOut = yIn - yIn.dot(xVec) * xVec;

	zOut = xOut.cross(yOut);

	double xx = xOut.getX();
	double yx = xOut.getY();
	double zx = xOut.getZ();

	double xy = yOut.getX();
	double yy = yOut.getY();
	double zy = yOut.getZ();

	double xz = zOut.getX();
	double yz = zOut.getY();
	double zz = zOut.getZ();

	tf::Matrix3x3 rotMatOut(xx, xy, xz, yx, yy, yz, zx, zy, zz);

	return rotMatOut;
}

