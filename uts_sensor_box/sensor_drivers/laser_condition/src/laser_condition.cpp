//////////////////////////////////////////////////////////////////////////////
//  Laser scanner conditioner                                               //
//                                                                          //
//  v.0.1                                                                   //
//  University of Technology, Sydney                                        //
//  lakshitha.dantanarayana@uts.edu.au                                      //
//////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

double angleleft_min,  angleleft_max,  angleright_min,  angleright_max;

using namespace std;
void scanCallback(const sensor_msgs::LaserScanConstPtr& laser);

ros::Publisher* _laser_out;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_cond");
	ros::NodeHandle n;

	ros::Publisher laser_out = n.advertise<sensor_msgs::LaserScan>("scan_out", 1000);
	_laser_out = &laser_out;

	n.param<double> ("angleleft_min", angleleft_min, -1.45);
	n.param<double> ("angleleft_max", angleleft_max, -1.25);
	n.param<double> ("angleright_min", angleright_min, 1.35);
	n.param<double> ("angleright_max", angleright_max, 1.55);

	if (!((angleleft_min<angleleft_max)&&(angleleft_max<angleright_min)&&(angleright_min<angleright_max)))
		ROS_ERROR("Angle intervals are incorrect, please check your params");

	ros::Subscriber laser_in = n.subscribe ( "scan_in",1000,scanCallback );

	cout << "Manipulating Data" << endl;

	ros::spin();

	cout << "terminating..." << endl;
}

void scanCallback(const sensor_msgs::LaserScanConstPtr& scan_in)
{
	int it=0;
	sensor_msgs::LaserScan scan_out;
	//ROS_INFO("Callback");

	scan_out.header = scan_in->header;
	scan_out.angle_min = scan_in->angle_min;
	scan_out.angle_max = scan_in->angle_max;
	scan_out.angle_increment = scan_in->angle_increment;
	scan_out.time_increment = scan_in->time_increment;
	scan_out.scan_time = scan_in->scan_time;
	scan_out.header = scan_in->header;
	scan_out.range_min = scan_in->range_min;
	scan_out.range_max = scan_in->range_max;
	scan_out.ranges = scan_in->ranges;


	for (double angle = scan_in->angle_min; angle <= scan_in->angle_max; angle += scan_in->angle_increment )
	{
		if(((angle>angleleft_min)&&(angle<angleleft_max) )||((angle>angleright_min)&&(angle<angleright_max)))
		  scan_out.ranges[it]=100;
		it++;
	}

_laser_out->publish(scan_out);
}