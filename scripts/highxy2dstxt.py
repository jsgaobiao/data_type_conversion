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

def formatAngle(theta) :
    twoPi = math.pi * 2.0
    if theta > twoPi :
        theta -= twoPi
    if theta < -twoPi :
        theta += twoPi
    return theta

def xy2Degree(x, y) :
    ret = 0
    if x > 0 and y < 0 :
        ret = math.atan(-y / x)
    elif x < 0 and y < 0 :
        ret = math.atan(x / y) + math.pi / 2.0
    elif x < 0 and y > 0 :
        ret = math.atan(y / (-x)) + math.pi
    elif x > 0 and y > 0 :
        ret = math.atan(x / y) + math.pi * 1.5
    ret = float(ret / math.pi * 180.0)
    # print (x, y)
    return 360 - ret

def remapLmsData(lmsData, pointNum) :
    lms = [32760 for i in range(1800)]
    for i in range(pointNum) :
        # degree = float(float(i) / float(pointNum)) * 360.0
        # (dis,) = struct.unpack('h', lmsData[i * 2 : i * 2 + 2])
        (x,) = struct.unpack('f', lmsData[i * 8 : i * 8 + 4])
        (y,) = struct.unpack('f', lmsData[i * 8 + 4 : i * 8 + 8])
        x = float(x)
        y = float(y)
        dis = float(math.sqrt(x * x + y * y)) * 100.0
        degree = xy2Degree(x, y)
        degree = float(degree)
        remapDegree = int(degree // 0.2)
        if remapDegree >= 1800 :
            remapDegree = 1799
        if lms[remapDegree] > dis :
            lms[remapDegree] = dis
    ret = ''
    for i in range(1800) :
        ret = ret + struct.pack('h', lms[i])
    return ret

if __name__ == '__main__':
    rospy.init_node('to_dstxt', anonymous=False)
    pGPS = 0
    cntGPS = 0

    fin = open('obstacle_outdoor_in_SAIC_04_06_plane.highxy', 'rb')
    tfin = open('obstacle_outdoor_in_SAIC.nav', 'r')
    fout = open('outdoor_04_06.txt', 'w')

    angRange = 360
    angRes   = 360.0 / 1799.0
    unit     = 100
    fout.write(str(angRange) + '\n')
    fout.write(str(angRes) + '\n')
    fout.write(str(unit) + '\n')

    line = tfin.readline()
    line = tfin.readline()
    lineData = line.split('\t')
    initRoll = float(lineData[1])
    initPitch = float(lineData[2])
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
        cnt = 0
        timeData = fin.read(4)
        if (not timeData) :
            break
        (timeData,) = struct.unpack('i', timeData)
        pointNum = fin.read(4)
        (pointNum,) = struct.unpack('i', pointNum)
        lmsData = fin.read(4 * 2 * pointNum)
        zeroData = fin.read(4 * 2 * (2200 - pointNum))
        # print pointNum
        lmsData = remapLmsData(lmsData, pointNum)

        # find matched GPS data
        while (pGPS < cntGPS) :
            gpsTime = float(gpsData[pGPS][0])
            gpsMilliTime = int(gpsTime // 1000 % 100000000)
            print gpsMilliTime
            if (gpsMilliTime > timeData) :
                break
            pGPS += 1

        # print timeData - gpsMilliTime
        roll = formatAngle(float(gpsData[pGPS][1]) - initRoll)
        pitch = formatAngle(float(gpsData[pGPS][2]) - initPitch)
        yaw = formatAngle(float(gpsData[pGPS][3]) - initYaw)
        x = float(gpsData[pGPS][4]) - initX
        y = float(gpsData[pGPS][5]) - initY
        z = float(gpsData[pGPS][6]) - initZ

        # ang = struct.pack('3d', roll, pitch, yaw)
        fout.write(str(roll) + ',' + str(pitch) + ',' + str(yaw) + ',')
        fout.write(str(x) + ',' + str(y) + ',' + str(z) + ',' + '0,')
        fout.write(str(gpsMilliTime) + '\n')
        for i in range(1800):
            lmsDataOne = lmsData[i*2:i*2+2]
            (lmsDataOne,) = struct.unpack('h', lmsDataOne)
            fout.write(str(lmsDataOne))
            if i != 1799 :
                fout.write(',')
        fout.write('\n')

    fin.close()
    tfin.close()
    fout.close()
