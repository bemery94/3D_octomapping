#!/usr/bin/env python

import message_filters
from sensor_msgs.msg import LaserScan, Imu
from tf.msg import tfMessage
import rospy
import rosbag
import time


class RecordRosBag(object):
    def __init__(self):
        self.my_bag = None
        self.create_bag()

    def create_bag(self):
        # Set up the bag file
        current_time = time.strftime('%H_%M_%S_%Y_%m_%d', time.localtime())
        bag_name = "sb_data_" + current_time + ".bag"
        print("Creating bag: {}".format(bag_name))
        try:
            self.my_bag = rosbag.Bag("/home/odroid/ros_bag" + bag_name, "w")
        except:
            self.my_bag = rosbag.Bag(bag_name, "w")

    def filter_callback(self, laser_scan_lsl, laser_scan_lsm, imu):
        self.my_bag.write('/scan_lsl', laser_scan_lsl)
        self.my_bag.write('/scan_lsm', laser_scan_lsm)
        self.my_bag.write('/myahrs_imu', imu)

    def tf_callback(self, data):
        self.my_bag.write('/tf', data)

    def __del__(self):
        self.my_bag.close()


def main():
    rospy.init_node('record_filtered_msgs')

    bag_ = RecordRosBag()

    # Subscribe to topics that need to be synchronised
    laser_lsl_sub = message_filters.Subscriber('/scan_lsl', LaserScan)
    laser_lsm_sub = message_filters.Subscriber('/scan_lsm', LaserScan)
    imu_sub = message_filters.Subscriber('/myahrs_imu', Imu)

    # Subscribe to other topics that need to be stored, but don't need to be synchronised
    rospy.Subscriber('/tf', tfMessage, bag_.tf_callback)

    # Apply message filter to only store messages that are within 0.15 seconds of each other
    filter_ = message_filters.ApproximateTimeSynchronizer([laser_lsl_sub, laser_lsm_sub, imu_sub], 10, 0.15)

    # Call class method to write synchronised messages to the bag file
    filter_.registerCallback(bag_.filter_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass