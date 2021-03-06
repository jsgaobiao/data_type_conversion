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

def remapLmsData(lmsData, pointNum) :
    lms = [32760 for i in range(1800)]
    for i in range(pointNum) :
        degree = float(float(i) / float(pointNum)) * 360.0
        remapDegree = int(degree // 0.2)
        if remapDegree >= 1800 :
            remapDegree = 1799
        (dis,) = struct.unpack('h', lmsData[i * 2 : i * 2 + 2])
        dis = int(dis)
        if lms[remapDegree] > dis :
            lms[remapDegree] = dis
    ret = ''
    for i in range(1800) :
        ret = ret + struct.pack('h', lms[i])
    return ret


if __name__ == '__main__':
    rospy.init_node('to_ds', anonymous=False)
    pGPS = 0
    cntGPS = 0

    fin = open('obstacle_indoor_parking.high', 'rb')
    tfin = open('indoorParking.traj', 'r')
    fout = open('indoorParking.ds', 'wb')

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
        cnt = 0
        timeData = fin.read(4)
        if (not timeData) :
            break
        (timeData,) = struct.unpack('i', timeData)
        pointNum = fin.read(4)
        (pointNum,) = struct.unpack('i', pointNum)
        lmsData = fin.read(2 * pointNum)
        zeroData = fin.read(2 * (2200 - pointNum))
        lmsData = remapLmsData(lmsData, pointNum)

        # find matched GPS data
        while (pGPS < cntGPS) :
            gpsTime = float(gpsData[pGPS][0])
            gpsMilliTime = int(gpsTime // 1000 % 100000000)
            if (gpsMilliTime > timeData) :
                break
            pGPS += 1

        print timeData
        roll = formatAngle(float(gpsData[pGPS][1]) - initRoll)
        pitch = formatAngle(float(gpsData[pGPS][2]) - initPitch)
        yaw = formatAngle(float(gpsData[pGPS][3]) - initYaw)
        x = float(gpsData[pGPS][4]) - initX
        y = float(gpsData[pGPS][5]) - initY
        z = float(gpsData[pGPS][6]) - initZ

        ang = struct.pack('3d', roll, pitch, yaw)
        fout.write(ang)
        shv = struct.pack('3d', x, y, z)
        fout.write(shv)
        gpsStat = struct.pack('B', 0)
        fout.write(gpsStat)
        timeStamp = struct.pack('i', gpsMilliTime)
        fout.write(timeStamp)
        fout.write(lmsData)

    fin.close()
    tfin.close()
    fout.close()
