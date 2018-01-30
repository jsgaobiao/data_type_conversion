#!/usr/bin/env python

import struct
import rospy
import sys
import string
import math
import tf
from sensor_msgs.msg._LaserScan import LaserScan
from sensor_msgs.msg._PointCloud2 import PointCloud2
from sensor_msgs.msg._PointField import PointField

def formatYaw(theta) :
    twoPi = math.pi * 2.0
    if theta > twoPi :
        theta -= twoPi
    if theta < -twoPi :
        theta += twoPi
    return theta

def initPointField() :
    pointFieldDatas = []
    pointFieldData = PointField()
    pointFieldData.name = 'x'
    pointFieldData.offset = 0
    pointFieldData.datatype = 7
    pointFieldData.count = 1
    pointFieldDatas.append(pointFieldData)
    pointFieldData = PointField()
    pointFieldData.name = 'y'
    pointFieldData.offset = 4
    pointFieldData.datatype = 7
    pointFieldData.count = 1
    pointFieldDatas.append(pointFieldData)
    pointFieldData = PointField()
    pointFieldData.name = 'z'
    pointFieldData.offset = 8
    pointFieldData.datatype = 7
    pointFieldData.count = 1
    pointFieldDatas.append(pointFieldData)
    pointFieldData = PointField()
    pointFieldData.name = 'intensity'
    pointFieldData.offset = 12
    pointFieldData.datatype = 7
    pointFieldData.count = 1
    pointFieldDatas.append(pointFieldData)
    return pointFieldDatas

def initVelo() :
    veloData = PointCloud2()
    veloData.header.seq = -1
    veloData.header.frame_id = 'velodyne'
    veloData.height = 1
    veloData.width = 513200  # 64*4*2200
    veloData.fields = initPointField()
    veloData.is_bigendian = False
    veloData.point_step = 16
    veloData.row_step = 513200 * 32
    veloData.is_dense = True
    veloData.data = []
    return veloData

if __name__ == '__main__':
    # fileIn = raw_input('Enter input file name : ')
    # fileOut = raw_input('Enter output file name : ')
    veloPub = rospy.Publisher('velodyne_points', PointCloud2, queue_size=10)
    tfBr = tf.TransformBroadcaster()
    rospy.init_node('lcm_publisher', anonymous=False)
    rate = rospy.Rate(100)  # hz
    veloData = initVelo()
    pGPS = 0
    cntGPS = 0

    fin = open('2017_06_07_14_45_15_atd.log.xyzi', 'rb')
    tfin = open('2017_06_07_14_45_15_atd.nav', 'r')

    line = tfin.readline()
    line = tfin.readline()
    lineData = line.split('\t')
    initYaw = float(lineData[3])
    initX = float(lineData[4])
    initY = float(lineData[5])
    initZ = float(lineData[6])

    gpsData = []
    while (True) :
        if (not line) :
            break
        line = tfin.readline()
        lineData = line.split('\t')
        gpsData.append(lineData)
        cntGPS += 1

    while not rospy.is_shutdown() :
        rate.sleep()
        nowStamp = rospy.Time.now()
        veloData.data = []
        cnt = 0

        timeData = fin.read(8)
        (timeData,) = struct.unpack('Q', timeData)

        for i in range(64) :
            pStr = 0
            finStr = fin.read(4+2200*3*4+2200*1)
            pointNumData = finStr[pStr:pStr+4]
            pStr += 4
            (pointNumData,) = struct.unpack('i', pointNumData)
            pointNumData = 100  # Debug
            cnt += pointNumData

            byteData = [[0 for p in range(3)] for j in range(2200)]
            intensityData = [0 for p in range(2200)]
            for j in range(2200) :
                for k in range(3) :
                    byteData[j][k] = finStr[pStr:pStr+4]
                    pStr += 4
                    (byteData[j][k],) = struct.unpack('f', byteData[j][k])
                intensityData[j] = finStr[pStr:pStr+1]
                pStr += 1
                (intensityData[j],) = struct.unpack('B', intensityData[j])

            for j in range(pointNumData) :
                packedPoint = struct.pack('@4f', byteData[j][0], byteData[j][1], byteData[j][2], float(intensityData[j]))
                # print byteData[j][0], byteData[j][1], byteData[j][2], intensityData[j]
                unpackedPoint = struct.unpack('16B', packedPoint)
                for k in range(len(unpackedPoint)) :
                    veloData.data.append(unpackedPoint[k])

        # fill scan data
        veloData.header.seq += 1
        print veloData.header.seq
        veloData.header.stamp = rospy.Time.from_sec(timeData / 1000000.0)
        veloData.width = cnt
        veloData.point_step = 16
        veloData.row_step = veloData.point_step * cnt
        veloPub.publish(veloData)

        # find matched GPS data
        while (pGPS < cntGPS) :
            gpsTime = float(gpsData[pGPS][0])
            if (gpsTime > timeData) :
                break
            pGPS += 1

        print timeData - gpsTime
        x = float(gpsData[pGPS][4]) - initX
        y = float(gpsData[pGPS][5]) - initY
        z = float(gpsData[pGPS][6]) - initZ
        yaw = formatYaw(float(gpsData[pGPS][3]) - initYaw)
        tfBr.sendTransform((x, y, 0),
                           tf.transformations.quaternion_from_euler(0, 0, -yaw),
                           rospy.Time.from_sec(timeData / 1000000.0),
                           'base_link',
                           'odom')
        tfBr.sendTransform((0, 0, 0),
                           tf.transformations.quaternion_from_euler(0, 0, 0),
                           rospy.Time.from_sec(timeData / 1000000.0),
                           'velodyne',
                           'base_link')
