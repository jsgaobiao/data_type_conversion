#!/usr/bin/env python

# This program is used to read data from .ds format
# and publish them, in which includes LaserScans, GPS shift (x, y, z), IMU (roll, pitch, yaw),
# GPS status and TimeStamp.

# .ds format example:
# 180.000000    // angle range
# 0.500000      // angle resolution
# 100.000000    // unit
# // roll,pitch,yaw,x,y,z,northaccuracy,eastaccuracy,downaccuracy,gpsstatus,lmsdat(TimeStamp+Scans)
# // data of one scan ...
# 2644,2634,2665,2779,2779,2787,2806,2654,2655,2648,2653,2664,2665,2200,2149,2671......

import rospy
import sys
import string
import math
import tf
import struct
from sensor_msgs.msg._LaserScan import LaserScan

def formatYaw(theta) :
    twoPi = math.pi * 2.0
    if theta > twoPi :
        theta -= twoPi
    if theta < -twoPi :
        theta += twoPi
    return theta

def initScan() :
    scanData = LaserScan()
    scanData.header.seq = -1
    scanData.header.frame_id = 'laser'
    scanData.angle_min = -math.pi / 2.0
    scanData.angle_max = math.pi / 2.0
    scanData.angle_increment = 0.003492599 # 360 / 1799 degree
    scanData.time_increment = 0 #1.73611151695e-05
    scanData.scan_time = 0.0250000003725
    scanData.range_min = 0.0230000000447
    scanData.range_max = 100
    return scanData


if __name__ == '__main__':
    # fileIn = raw_input('Enter input file name : ')
    # fileOut = raw_input('Enter output file name : ')
    scanPub = rospy.Publisher('scan', LaserScan, queue_size=10)
    # tfBr = tf.TransformBroadcaster()
    rospy.init_node('ds_publisher', anonymous=False)
    rate = rospy.Rate(10)  # hz
    scanData = initScan()

    if len(sys.argv) != 2 :
        print '[Usage : pythone ds_publisher.py [input file name]'
        sys.exit()
    else :
        fileIn = sys.argv[1]
        # fileOut = sys.argv[2]

    fin = open(fileIn, 'rb')
    angRange, angRes, unit = struct.unpack("fff", fin.read(4 * 3))
    laserPointNums = int(angRange / angRes + 1)
    laserByteNums = 2 * int(laserPointNums)
    laserScan = [0] * int(laserPointNums)

    # get the initial data
    initRoll, initPitch, initYaw = struct.unpack("ddd", fin.read(8 * 3))
    initX, initY, initZ = struct.unpack("ddd", fin.read(8 * 3))
    initNorthAc, initEastAc, initDownAc = struct.unpack("fff", fin.read(4 * 3))
    gpsStatus, = struct.unpack("f", fin.read(4))
    lastTimeStamp, = struct.unpack("q", fin.read(8))
    fin.read(laserByteNums)

    while not rospy.is_shutdown() :
        rate.sleep()
        nowStamp = rospy.Time.now()

        line = fin.read(8 * 3)
        if (not line) :
            break
        roll, pitch, yaw = struct.unpack("ddd", line)
        x, y, z = struct.unpack("ddd", fin.read(8 * 3))
        northAc, eastAc, downAc = struct.unpack("fff", fin.read(4 * 3))
        gpsStatus, = struct.unpack("f", fin.read(4))
        timeStamp, = struct.unpack("q", fin.read(8))
        for i in range(laserPointNums):
            laserScan[i], = struct.unpack("h", fin.read(2))

        # tfBr.sendTransform((x, y, 0),
        #                    tf.transformations.quaternion_from_euler(0, 0, yaw),
        #                    nowStamp,
        #                    'base_link',
        #                    'odom')
        # tfBr.sendTransform((0, 0, 0),
        #                    tf.transformations.quaternion_from_euler(0, 0, 0),
        #                    nowStamp,
        #                    'laser',
        #                    'base_link')

        # fill scan data
        scanData.header.seq += 1
        scanData.header.stamp = nowStamp
        scanData.ranges = []
        cnt = 0
        for scanRange in laserScan :
            cnt += 1
            if cnt > 900:
                break
            scanData.ranges.append(float(scanRange) / float(unit))
        scanPub.publish(scanData)
