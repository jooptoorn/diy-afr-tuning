#!/usr/bin/env python
import serial
import os

serDevPath = '/dev/ttyACM0'
logFolder = 'afrlogs/'
# serDevPath = 'COM12'
# logFolder = 'C:\\Users\\joopt\\OneDrive\\motor\\rsv mille\\tuning\\logger\\testing\\sim_logfiles\\'

ser = serial.Serial(serDevPath, 115200)

# make sure we use a unique filename and dont over-write previous logfiles
logFileNum = 1
logPreamble = 'afr_log'
logFp = logFolder + logPreamble + str(logFileNum) + '.log'

while(os.path.exists(logFp)):
    logFileNum += 1
    logFp = logFolder + logPreamble + str(logFileNum) + '.log'

# found a unique logfile name
print("Writing to log " + logFp)
file = open(logFp,'xb')

while(1):
    serData = ser.readline()
    # serTextData = serData.decode('utf-8')
    # print(serTextData)
    file.write(serData)
    file.flush()

file.close()
