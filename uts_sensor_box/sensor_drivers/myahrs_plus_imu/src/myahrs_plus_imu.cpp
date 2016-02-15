#include "myahrs_plus_imu.hpp"

class SensorBoxIMU : public iMyAhrsPlus
{
    Platform::Mutex lock;
    SensorData sensor_data;

public:
    int sample_count;
    bool new_data;

    SensorBoxIMU(std::string port="", unsigned int baudrate=57600)
    : iMyAhrsPlus(port, baudrate), sample_count(0), new_data(false) {}
    ~SensorBoxIMU() {}

    bool initialize() {
        bool ok = false;
        do {
            if(start() == false) break;
            if(cmd_binary_data_format("QUATERNION, IMU") == false) break;
            if(cmd_divider(DIVIDER) == false) break;
            if(cmd_mode("BC") == false) break;
            ok = true;
        } while(0);

        return ok;
    }

    inline void get_data(SensorData& data) {
        LockGuard _l(lock);
        data = sensor_data;
    }

    inline SensorData get_data() {
        LockGuard _l(lock);
        return sensor_data;
    }

    void publish_data() {
        Quaternion& q = sensor_data.quaternion;
        EulerAngle& e = sensor_data.euler_angle;
        ImuData<float>& imu = sensor_data.imu;

    	imu_data_.orientation.x = q.x;
	imu_data_.orientation.y = q.y;
	imu_data_.orientation.z = q.z;
	imu_data_.orientation.w = q.w;
	imu_data_.angular_velocity.x = imu.gx;
	imu_data_.angular_velocity.y = imu.gy;
	imu_data_.angular_velocity.z = imu.gz;
	imu_data_.linear_acceleration.x = imu.ax;
	imu_data_.linear_acceleration.y = imu.ay;
	imu_data_.linear_acceleration.z = imu.az;
	imu_data_.header.frame_id = "imu";
	imu_data_.header.stamp = ros::Time::now();

    myahrs_pub.publish(imu_data_);
    }

protected:
    /*
     * 	override event handler
     */
    void OnSensorData(int sensor_id, SensorData data) {
        sample_count++;
        {
            LockGuard _l(lock);
            sensor_data = data;
            sensor_data.euler_angle = sensor_data.quaternion.to_euler_angle();
        }
    }

    void OnAttributeChange(int sensor_id, std::string attribute_name, std::string value) {
        printf("OnAttributeChange(id %d, %s, %s)\n", sensor_id, attribute_name.c_str(), value.c_str());
    }
};

//main ros_node function
int main(int argc, char **argv)
{
	//ros initialisation
	ROS_INFO("Starting IMU Node");
	ros::init(argc,argv, "myahrs_node");
	ros::NodeHandle nh;
	myahrs_pub = nh.advertise<sensor_msgs::Imu>(imu_topic_, 1);

	nh.param<std::string>("imu_port", serial_device, "/dev/sb/imu");

	SensorBoxIMU sensor(serial_device, baudrate);

	if(sensor.initialize() == false)
		ROS_ERROR("Failed to initialise IMU. Check baudrate & dev settings.");

	ROS_INFO("Initialisation Complete. Publishing to myahrs_imu.");

	//ros::spin();
	ros::Rate rate(50);
    	while(ros::ok())
	{
		sensor.publish_data();
		rate.sleep();
	}

	ROS_INFO("Closing IMU Comms");
	sensor.stop();
}
