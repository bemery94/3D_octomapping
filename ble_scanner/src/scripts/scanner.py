#!/usr/bin/env python

import rospy
import blescan
import sys
import bluetooth._bluetooth as bluez
from ble_scanner.msg import *


class BLEScannerNode():
    def __init__(self):
        rospy.init_node("blescanner", anonymous=True)

        dev_id = 0
        try:
            self.sock = bluez.hci_open_dev(dev_id)
            print "ble thread started"
        except:
            print "error accessing bluetooth device..."
            sys.exit(1)

        blescan.hci_le_set_scan_parameters(self.sock)
        blescan.hci_enable_le_scan(self.sock)

        self.pub = rospy.Publisher('ble_data', BLEData, queue_size=10, latch=True)

        while not rospy.is_shutdown():
            self.returnedList = blescan.parse_events(self.sock, 10)
            msg = BLEData()
            msg.header.stamp = rospy.Time.now()
            for beacon in self.returnedList:
                submsg = BLEBeacon()
                submsg.mac_address = beacon[0:17]
                submsg.rssi = int(beacon[-3:])
                msg.data.append(submsg)
                # print submsg
                # print "MAC ADDRESS: ", beacon[0:17]
                # print "RSSI: ", beacon[-3:]
                # print beacon

            self.pub.publish(msg)


if __name__ == "__main__":
    try:
        bleScanner = BLEScannerNode()
    except rospy.ROSInterruptException:
        pass