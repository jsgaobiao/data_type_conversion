#!/usr/bin/env python

# This program is used to read data from .dsetxt format (a text version of .des file)
# and publish them, in which includes LaserScans, GPS shift (x, y, z), IMU (roll, pitch, yaw),
# GPS status and TimeStamp.

# .dsetxt format example:
# 180.000000    // angle range
# 0.500000      // angle resolution
# 100.000000    // unit
# // encoder,roll,pitch,yaw,x,y,z,GPS status,TimeStamp (millisecond)
# 0,-0.003393,0.005329,-3.098673,26375.889000,-1421.077000,-78.393560,7,23400002
# // data of one scan ...
# 2644,2634,2665,2779,2779,2787,2806,2654,2655,2648,2653,2664,2665,2200,2149,2671......

import rospy
import sys
import string
import math
import tf
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
    scanData.angle_min = -math.pi # / 2.0
    scanData.angle_max =  math.pi # / 2.0
    scanData.angle_increment = 0.003492599  # 360 / 1799 degree
    scanData.time_increment = 1.73611151695e-05
    scanData.scan_time = 0.0250000003725
    scanData.range_min = 0.0230000000447
    scanData.range_max = 100
    return scanData


if __name__ == '__main__':
    # fileIn = raw_input('Enter input file name : ')
    # fileOut = raw_input('Enter output file name : ')
    scanPub = rospy.Publisher('scan', LaserScan, queue_size=10)
    tfBr = tf.TransformBroadcaster()
    rospy.init_node('ds_publisher', anonymous=False)
    rate = rospy.Rate(40)  # hz
    scanData = initScan()

    if len(sys.argv) != 2 :
        print '[Usage : pythone ds_publisher.py [input file name]'
        sys.exit()
    else :
        fileIn = sys.argv[1]
        # fileOut = sys.argv[2]

    fin = open(fileIn, 'r')
    angRange = float(fin.readline())
    angRes = float(fin.readline())
    unit = float(fin.readline())

    # get the initial data
    # for i in range(6800) :  # skip datas that the car dosen't move
    #     line0 = fin.readline()
    #     line1 = fin.readline()
    line0 = fin.readline()
    line1 = fin.readline()
    line0 = line0.split(',')
    line1 = line1.split(',')
    initX = float(line0[3])
    initY = float(line0[4])
    initYaw = float(line0[2])
    lastTimeStamp = float(line0[7])
    scanNum = len(line1)


    while not rospy.is_shutdown() :
        rate.sleep()
        nowStamp = rospy.Time.now()
        # encoder,roll,pitch,yaw,x,y,z,GPS status,TimeStamp (millisecond)
        line0 = fin.readline()
        # scan datas
        line1 = fin.readline()
        if (not line0) or (not line1) :
            break
        line0 = line0.split(',')
        line1 = line1.split(',')

        # get (x, y, yaw) and broadcast to /tf  ( odom -> base_link)
        x = float(line0[3]) - initX
        y = float(line0[4]) - initY
        yaw = formatYaw(float(line0[2]) - initYaw)
        # yaw += 0.09424778
        tfBr.sendTransform((-y, x, 0),
                           tf.transformations.quaternion_from_euler(0, 0, -yaw),
                           nowStamp,
                           'base_link',
                           'odom')
        tfBr.sendTransform((0, 0, 0),
                           tf.transformations.quaternion_from_euler(0, 0, 0),
                           nowStamp,
                           'laser',
                           'base_link')

        # fill scan data
        scanData.header.seq += 1
        scanData.header.stamp = nowStamp
        scanData.ranges = []
        for scanRange in line1 :
            scanData.ranges.append(float(scanRange) / unit)
        scanPub.publish(scanData)
