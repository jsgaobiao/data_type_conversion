#!/usr/bin/env python

import struct
import rospy
import sys
import string
import math
import time
import tf
import pdb
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
    return 360 - ret

def remapLmsData(lmsData, pointNum) :
    lms = [32760 for i in range(1800)]
    for i in range(pointNum) :
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
    # print lms
    for i in range(1800) :
        ret = ret + struct.pack('h', lms[i])
    return ret


if __name__ == '__main__':
    rospy.init_node('to_ds', anonymous=False)
    pGPS = 0
    cntGPS = 0

    fin = open('outdoor_20_22_0726.highxy', 'rb')
    tfin = open('outdoor_0726.nav', 'r')
    fout = open('Anting_20_22_0726.ds', 'wb')

    angRange = struct.pack('f', 360)
    angRes   = struct.pack('f', 360 / 1799.0)
    unit     = struct.pack('f', 100)
    fout.write(angRange)
    fout.write(angRes)
    fout.write(unit)

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
        # time.sleep(1)
        cnt = 0
        timeData = fin.read(4)
        if (not timeData) :
            break
        (timeData,) = struct.unpack('i', timeData)
        pointNum = fin.read(4)
        (pointNum,) = struct.unpack('i', pointNum)
        lmsData = fin.read(4 * 2 * pointNum)
        if len(lmsData) != pointNum*8 :
            break
        zeroData = fin.read(4 * 2 * (2200 - pointNum))
        if len(zeroData) != (2200 - pointNum)*8 :
            break
        lmsData = remapLmsData(lmsData, pointNum)

        # find matched GPS data
        while (pGPS < cntGPS) :
            gpsTime = float(gpsData[pGPS][0])
            gpsMilliTime = int(gpsTime // 1000 % 100000000)
            # gpsMilliTime = int(gpsTime)
            if (gpsMilliTime > timeData) :
                break
            pGPS += 1

        # print timeData, pointNum
        roll = formatAngle(float(gpsData[pGPS][1]) - initRoll)
        pitch = formatAngle(float(gpsData[pGPS][2]) - initPitch)
        yaw = formatAngle(float(gpsData[pGPS][3]) - initYaw)
        x = float(gpsData[pGPS][4]) - initX
        y = float(gpsData[pGPS][5]) - initY
        z = float(gpsData[pGPS][6]) - initZ
        # print ("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f" % (roll, pitch, yaw, x*100, y*100, z*100))

        ang = struct.pack('3d', roll, pitch, yaw)
        fout.write(ang)
        shv = struct.pack('3d', y, x, z)
        fout.write(shv)
        gpsStat = struct.pack('B', 0)
        fout.write(gpsStat)
        timeStamp = struct.pack('i', gpsMilliTime)
        fout.write(timeStamp)
        fout.write(lmsData)

    fin.close()
    tfin.close()
    fout.close()
