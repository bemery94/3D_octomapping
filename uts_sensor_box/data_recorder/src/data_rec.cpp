#include "data_rec.hpp"

using namespace Sensor_Box_pkg;

void initialise()
{
	//initialise ros and node handle
	ros::NodeHandle nh;

	//initialise necessary parameters
	nh.param("Hz", HZ, 1);
	nh.param<double>("cam_timer", m_device_timer[CAM], 1.0);
	nh.param<double>("imu_timer", m_device_timer[IMU], 1.0);
	nh.param<double>("lsl_timer", m_device_timer[LASER_LSL],1.0);
	nh.param<double>("lsm_timer", m_device_timer[LASER_LSM], 1.0);
	nh.param<double>("disk_timer", m_device_timer[DISK_MNTR], 5.0);

	//setup ros subscribers
	sub_imu = nh.subscribe(imu_topic_, 1, IMU_cb);
	imu_timer = nh.createTimer( ros::Duration(0.5), IMU_timer_cb);

	sub_lsl = nh.subscribe(lsl_topic_, 1, LSL_cb);
	lsl_timer = nh.createTimer( ros::Duration(0.5), LSL_timer_cb);

	sub_lsm = nh.subscribe(lsm_topic_, 1, LSM_cb);
	lsm_timer = nh.createTimer( ros::Duration(0.5), LSM_timer_cb);

	sub_cam = nh.subscribe(cam_topic_, 1, CAM_cb);
	cam_timer = nh.createTimer( ros::Duration(0.5), CAM_timer_cb);

	sub_dis = nh.subscribe(dis_topic_, 1, DISK_cb);
	cam_timer = nh.createTimer( ros::Duration(5.0), DISK_timer_cb);

	sub_tfs = nh.subscribe(tfs_topic_, 1, TFS_cb);

	sub_usr = nh.subscribe(usr_topic_, 1, USR_cb);

	//setup ros publishers
	pub_usr_ = nh.advertise<std_msgs::UInt8>(usr_wtopic_, 1000);
	pub_dis_ = nh.advertise<data_recorder::Fs_space>(dis_topic_,1000);

	m_recordobj = new Record ( &nh );

	ROS_INFO("Configuration complete, running main loop");
}

int main(int argc, char **argv)
{
	rec_status_ = INITIALISING;
	ros::init(argc, argv, "data_recorder");
	initialise();

	//monitor thread - check the status of all the sensors every 30 seconds
	boost::thread thread1 ( &monitor );
	//process thread - passes the data over to the recording object which handles recording
	boost::thread thread2 ( &process );

	struct statvfs fs;
	data_recorder::Fs_space diskMsg;

	//this thread monitors the disk space
	ros::Rate rate(30);
	rec_status_ = READY;
	while( !m_exitThread)
	{
		m_exitThread = !ros::ok();
	
		statvfs ( "/", &fs );
		diskMsg.fs_size = fs.f_blocks*fs.f_bsize/1000000.0;
   		diskMsg.fs_space = fs.f_bsize*fs.f_bavail/1000000.0;
    	diskMsg.header.stamp = ros::Time::now();
        pub_dis_.publish ( diskMsg );

        //ROS_INFO("Diskspace: %f" , fs.f_bavail/(float)fs.f_blocks *100.0);
        if ((fs.f_bavail/(float)fs.f_blocks *100.0) <10.0)
        {
            std::ofstream diskfull;

            diskfull.open("/etc/sensor_box/disk_full");
            diskfull << "full";
            diskfull.close();
	    	rec_status_ = LOW_MEMORY;
  		}

		if(start_rec)
		{
			m_guardrecord.lock();
			//check sensor status before starting recording
			 if ( ( m_sensorstatus & ALL_SENSORS ) == ALL_SENSORS ) 
			{
				m_recordflag = true;
				ROS_INFO("[main loop] Started recording");
				ROS_INFO("Recording devices - LSM = %0.1f - CAM = %0.1f", (double)rec_lsm, (double)rec_cam);
				start_rec=0;
				rec_status_ = RECORDING;
				m_recordobj->newBag();
			}
			m_guardrecord.unlock();
		}

		else if(stop_rec)
		{
			m_guardrecord.lock();
			m_recordflag = false;
			m_guardrecord.unlock();
			ROS_INFO("[main loop] Stopped recording");
			rec_status_ = READY;
			stop_rec = 0;
			m_recordobj->closeBag();
			//close the recording - destroy the record object?
			//bag.close();
		}

		ros::spinOnce();
		rate.sleep();
	}
	//if(bopen_)	bag.close();
	//thread1.join(); thread2.join();
}

void process()
{
	ROS_INFO("[process] Started processing sensor callbacks");
    baseMsg* msg_ ;

    while (!m_exitThread )
    {
        m_guard.lock();
        if ( m_msgqueue.empty() )
        {
            m_guard.unlock();
            usleep(100);
        }
        else
        {
            msg_ = m_msgqueue.front();
            m_msgqueue.pop();
            m_guard.unlock();
            m_recordobj->record ( msg_ );
        }
    }
}

void monitor()
{
    ROS_INFO ( "[monitor] Started monitoring sensors" );
    bool healthstat = false;
    bool first_start = true;

	while ( !m_exitThread ) 
	{
	    healthstat = false;
	    m_guardrecord.lock();

	    //check whether all sensors are being used
	    ALL_SENSORS = ALLSENSORS_ON;
	    if(!rec_lsm)
	        ALL_SENSORS &= ~LASER_LSM_ON;
	    if(!rec_cam)
	        ALL_SENSORS &= ~CAM_ON;
	    if(!rec_disk)
	    	ALL_SENSORS &= ~DISK_ON;

	    ROS_INFO ( "[monitor] LSL = %d, LSM = %d, CAM = %d, IMU = %d" ,
	    (m_sensorstatus & LASER_LSL_ON ) > 0,
	    (m_sensorstatus & LASER_LSM_ON ) > 0,
	    (m_sensorstatus & CAM_ON) > 0,
	    (m_sensorstatus & IMU_ON) > 0 );

	    //check the health of the sensors
	    if ( ( m_sensorstatus & ALL_SENSORS ) == ALL_SENSORS ) 
	        healthstat = true;

	    m_guardrecord.unlock();

	    //clear m_sensor_status flag to be set by the callbacks
	    if ( healthstat || first_start ) 
	        m_sensorstatus = 0;
	    else 
	    {
	        ROS_ERROR ( "System health check failed: %x",m_sensorstatus );
	        m_sensorstatus = 0;
		rec_status_ = SENSOR_ERROR;
	    }
	    sleep ( 10 );
	    first_start = false;
    }
}

void IMU_cb(const sensor_msgs::ImuConstPtr& msg)
{
	bool recordstatus;
	bool m_dev;
    m_guardrecord.lock();
    recordstatus = m_recordflag;
    m_sensorstatus = m_sensorstatus | IMU_ON;
    m_guardrecord.unlock();

    m_device[IMU] = true;

    if ( ( m_device[IMU] ) && ( recordstatus ) && ( !m_exitThread ) ) 
    {
        imuRealMsg* imurealmsg = new imuRealMsg ( imu_topic_ );
        imurealmsg->setMessage ( msg );

        m_guard.lock();
        m_msgqueue.push ( imurealmsg );

        m_guard.unlock();
    }
}

void LSL_cb(const sensor_msgs::LaserScanConstPtr& msg)
{
	bool recordstatus;

    m_guardrecord.lock();
    recordstatus = m_recordflag;
    m_sensorstatus = m_sensorstatus | LASER_LSL_ON;
    m_guardrecord.unlock();

    m_device[LASER_LSL] = true;

    if ( ( m_device[LASER_LSL] ) && ( recordstatus ) && ( !m_exitThread ) ) 
    {
        laserLSLMsg* lasermsg = new laserLSLMsg ( lsl_topic_ );
        lasermsg->setMessage ( msg );

        m_guard.lock();
        m_msgqueue.push ( lasermsg );

        m_guard.unlock();
    }
}

void LSM_cb(const sensor_msgs::LaserScanConstPtr& msg)
{
	bool recordstatus;

    m_guardrecord.lock();
    recordstatus = m_recordflag;
    m_sensorstatus = m_sensorstatus | LASER_LSM_ON;
    m_guardrecord.unlock();

   m_device[LASER_LSM] = true;

    if ( (rec_lsm) && ( m_device[LASER_LSM] ) && ( recordstatus ) && ( !m_exitThread ) ) 
    {
        laserLSMMsg* lasermsg = new laserLSMMsg ( lsm_topic_ );
        lasermsg->setMessage ( msg );

        m_guard.lock();
        m_msgqueue.push ( lasermsg );

        m_guard.unlock();
    }
}

void CAM_cb(const sensor_msgs::ImageConstPtr& msg)
{
	bool recordstatus;

    m_guardrecord.lock();
    recordstatus = m_recordflag;
    m_sensorstatus = m_sensorstatus | CAM_ON;
    m_guardrecord.unlock();

    m_device[CAM] = true;

    if ( (rec_cam) && ( m_device[CAM] ) && ( recordstatus ) && ( !m_exitThread ) ) 
    {
        imageMsg* imagemsg = new imageMsg ( cam_topic_ );
        imagemsg->setMessage ( msg );

        m_guard.lock();
        m_msgqueue.push ( imagemsg );

        m_guard.unlock();
    }
}

//callback for reporting current disk space
void DISK_cb ( const data_recorder::Fs_spaceConstPtr& disk_stat )
{
    bool recordstatus;

    m_guardrecord.lock();
    recordstatus = m_recordflag;
    m_sensorstatus = m_sensorstatus | DISK_ON;
    m_guardrecord.unlock();

    m_device[DISK_MNTR] = true;

    if ( (rec_disk) && ( m_device[DISK_MNTR] ) && ( recordstatus ) && ( !m_exitThread ) ) {
        ROS_WARN( "current msg queue len= %d" , m_msgqueue.size()  );
        fsMntrMsg* fsmntrmsg = new fsMntrMsg ( dis_topic_ );
        fsmntrmsg->setMessage ( disk_stat );

        m_guard.lock();
        m_msgqueue.push ( fsmntrmsg );
        m_guard.unlock();
    }
}

//callback for recording transforms
void TFS_cb(const tf::tfMessageConstPtr& tf_m)
{
	bool recordstatus;

	m_guardrecord.lock();
    recordstatus = m_recordflag;
    m_guardrecord.unlock();

	if(!m_exitThread && recordstatus)
	{
		tfMsg* tfmsg = new tfMsg("tf");
		tfmsg->setMessage(tf_m);

		m_guard.lock();
		m_msgqueue.push(tfmsg);
		m_guard.unlock();
	}
}

void USR_cb(const std_msgs::UInt8& msg)
{
	usr_ = msg;		//copy data into global variable
	newusr_ = 1;	//raise flag to indicate new data

	//handle user inputs/indicators
	user_io();
}

void user_io()
{
	//check available switches and buttons
	if(newusr_)
	{
		check_input();
		newusr_ = 0;
	}

	//output state to io_write topic
	output_status();
}

void check_input()
{
	int cmd = usr_.data;

	//start/stop recording - falling edge check (release of button)
	if(((prev_cmd & B_USER) > 0) && !((cmd & B_USER) > 0))
	{
		start_rec = (rec_status_ == READY) ? 1 : 0;
		stop_rec = (rec_status_ == RECORDING) ? 1 : 0;
	}

	//store the prev command for falling edge check on start-stop button
	prev_cmd = cmd;
	
	//record mapping laser scanner (localisation one required by default)
	rec_lsm = ((cmd & SW_LSM) > 0) ? 1 : 0;

	//record camera images
	rec_cam = ((cmd & SW_CAM) > 0) ? 1 : 0;
}

void output_status()
{
	io_write_.data = rec_status_;

	pub_usr_.publish(io_write_);
}

void IMU_timer_cb(const ros::TimerEvent& event)
{
	m_device[IMU] = true;
}

void CAM_timer_cb(const ros::TimerEvent& event)
{
	m_device[CAM] = true;
}

void LSL_timer_cb(const ros::TimerEvent& event)
{
	m_device[LASER_LSL] = true;
}

void LSM_timer_cb(const ros::TimerEvent& event)
{
	m_device[LASER_LSM] = true;
}

void DISK_timer_cb(const ros::TimerEvent& event)
{
	m_device[DISK_MNTR] = true;
}

std::string get_curr_time()
{
  char buff[20]; time_t now = time(NULL);
  strftime(buff, 20, "%Y-%m-%d-%H-%M-%S", localtime(&now));
  return std::string(buff);
}
