#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"

/** @file initial_tf_pub.cpp
 *  @brief Calculates and publishes the initial imu offset and and also the transform between the
 *  map and inertial frame (both at time t=0).
 *
 *  For more information, see coordinate_frame_info.tex in the documentation folder in the root of
 *  the package.
 *
 *  NOTE. If IMU is changed, then the relative transform rotInertialToCorrImu needs to be changed
 *  in this node according to the new inertial frame of the IMU.
 *
 *  @author Brendan Emery
 *  @date Feb 2016
 *  @version 1.0.0
 *  @bug Currently no known bugs.
 *  @todo Currently no todos.
 */

// Global variables
sensor_msgs::Imu localMsg;
tf::Transform transformCorrImuToImu;
tf::Transform transformInertialToMap;
bool firstMsg;
bool tfCalculated;
tf::TransformListener* listener_ = NULL;

// Functional Prototypes
void imuCallBack(const sensor_msgs::Imu::ConstPtr& msg);
tf::Matrix3x3 getRotationMat(std::string, std::string);
tf::Matrix3x3 convEulerToRot(double, double, double);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "initial_tf_pub");
	ros::NodeHandle n;

	listener_ = new tf::TransformListener();
	firstMsg = true;
	tfCalculated = false;

    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("myahrs_imu", 100, imuCallBack);

	ros::Rate rate(40);
	while(ros::ok())
	{
		/*
		 * Only perform computations the first time. We only want the transforms at time t=0 so
		 * repeating the computations in every loop is unnecessary.
		*/
		if(!tfCalculated) {
			////// Calculate the transform between the inertial frame and map frame: //////

			////////////////////////////////////// User Set ////////////////////////////////////////
			/*
			 * User should set here the roll, pitch and yaw values that transform the IMU into the
			 * inertial frame. This may need to be changed if a new IMU is used. Note. the values
			 * are in radians.
			 */
			double roll = 0;
			double pitch = 0;
			double yaw = 0;
			////////////////////////////////////////////////////////////////////////////////////////

			// Store the rpy values from above in a rotation matrix.
			tf::Matrix3x3 rotInertialToCorrImu;
			rotInertialToCorrImu.setEulerYPR(yaw, pitch, roll);

			tf::Matrix3x3 rotCorrImuToBl;
			rotCorrImuToBl = getRotationMat("/corrected_imu", "/base_link");

			tf::Matrix3x3 rotInertialToMap;
			rotInertialToMap = rotInertialToCorrImu * rotCorrImuToBl;

			////// Calculate the transform between the corrected imu and the imu frame: //////

			tf::Quaternion quatInertialToImu;

			// Stores the quaternion from the imu in a tf quaternion so that it can be converted
			// to a rotation matrix
			tf::quaternionMsgToTF(localMsg.orientation, quatInertialToImu);

			tf::Matrix3x3 rotInertialToImu(quatInertialToImu);

			tf::Matrix3x3 rotCorrImuToImu;
			rotCorrImuToImu = rotInertialToCorrImu.transpose() * rotInertialToImu;

			////// Broadcast the 2 transforms //////

			transformCorrImuToImu.setBasis(rotCorrImuToImu);

			transformInertialToMap.setBasis(rotInertialToMap);

			// Set the translation between origins to be 0
			transformCorrImuToImu.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

			transformInertialToMap.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
		}

		// Publish the same transform at a consistent rate
	    tf::StampedTransform transformOut1(transformCorrImuToImu, ros::Time::now(),
	                                      "/corrected_imu",
	                                      "/imu");

		tf::StampedTransform transformOut2(transformInertialToMap, ros::Time::now(),
	                                      "/inertial",
	                                      "/map_world_frame");

	    static tf::TransformBroadcaster br;
	    br.sendTransform(transformOut1);
	    br.sendTransform(transformOut2);

		rate.sleep();
		ros::spinOnce();
	}
}


void imuCallBack(const sensor_msgs::Imu::ConstPtr& msg)
/*
 * This callback calculates and publishes The callback should only
 */
{
	if(firstMsg)
	{
		localMsg = *msg;
		firstMsg = false;
	}
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
