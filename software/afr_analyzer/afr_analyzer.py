import tkinter as tk
import re
import numpy as np
import matplotlib.pyplot as plt

# setting grid on here is easiest
plt.rcParams['axes.grid'] = True

logFilePath = "C:\\Users\\joopt\\OneDrive\\motor\\rsv mille\\tuning\\logger\\testing\\20240520 first ride no accel pump"
logFileFile = "ride_home.log"
# logFilePath = "C:\\Users\\joopt\\OneDrive\\motor\\1st-ride-w-max-rev.log"

# fuel table throttle position columns in fuel table of PCIII
ftTpVals = [0.0, 2.0, 5.0, 10.0, 20.0, 40.0, 60.0, 80.0, 100.0]

# fuel table RPM rows in fuel table of PCIII
# note 1: the max rpm of the V990 rotax engine is 10.500, even though the PCIII stores values up to 12k. We will omit analysis of 10.5k - 12k
# note 2: a low-res fuel map (default) has one row per 500rpm, while a his-res map has one row per 250rpm.
ftRpmVals = [x * 1.0 for x in range(500, 10500, 500)]

# parameters for extraction of log entries for analysis in fuel table
# extract values around TPS value in range around a percentage [target*(1-val), target*(1+val)]
# with a hard limit on +/- X% over or under the target value
ftTpScaleLim = 0.25   # example: extract all values around 5% with ftTpScaleLim = 0.25. Script will count all values in range [5%*0.75, 5%*1.25] = [3.75, 6.25]
ftTpValLim = 10.0     # example: extract all values around 80% with ftTpScaleLim = 0.5 i.e. range [40%, 120%] with ftTpValLim = 10 ==> range narrowed to [70%, 90%]

# since rpm rows always have fixed distance of either 500rpm or 250rpm, just use fixed value here
ftRpmValLim = 200.0   # example: extract all values around 5.000rpm with ftRpmValLim=200.0 => range is [4.800 5.200]

# function to extract all values around a certain fuel table cell defined by a combination of tps and rpm value
def extractLogVals(tps, rpm, ld):
    # value check before attempting to extract log values
    if(tps<ftTpVals[0] or tps >ftTpVals[-1] or rpm<ftRpmVals[0] or rpm>ftRpmVals[-1]):
        print("Error: extracting log values with unusable tps or rpm values")
        print("tps=" + str(tps) + "\t rpm=" + str(rpm))
        return

    # format of logdata (ld): [rpm, tps, afr]

    # first extract all values limited by scaling limit for tps
    exstrLdIdx = np.where(np.logical_and(ld[:,1] > tps*(1-ftTpScaleLim), ld[:,1] < tps*(1+ftTpScaleLim)))
    exstrLdRows = ld[exstrLdIdx]

    # throw away values that fall outside of hard limit range for tps
    exstrLdIdx = np.where(np.logical_and(exstrLdRows[:,1] > tps-ftTpValLim, exstrLdRows[:,1] < tps+ftTpValLim))
    exstrLdRows = exstrLdRows[exstrLdIdx]

    # throw away values that fall outside of target rpm range
    exstrLdIdx = np.where(np.logical_and(exstrLdRows[:,0] > rpm-ftRpmValLim, exstrLdRows[:,0] < rpm+ftRpmValLim))
    exstrLdRows = exstrLdRows[exstrLdIdx]
    
    return

def readLogData(fp):
    with open(fp, newline='\r\n') as csvfile:
        # uses numpy array because statistical analysis on afr data will be faster that way
        #ld = log data
        ld = np.zeros([1,3])

        for row in csvfile:
            rowData = re.split(' |,|=|\t|\r\n', row)
            # print(rowData)
            # check if we should use this data
            # check if all expected field are there (RPM,AFR,STATUS,TP)
            valDetected = all(x in rowData for x in ['RPM', 'AFR', 'STATUS', 'TP'])
            if(valDetected):
                # check if the AFR controller status was OK
                i = rowData.index('STATUS') + 1
                afrStatusVal = rowData[i]
                if(afrStatusVal == 'OK'):
                    # the AFR data is valid, read the rest
                    i = rowData.index('RPM') + 1
                    rpmVal = float(rowData[i])

                    i = rowData.index('AFR') + 1
                    afrVal = float(rowData[i])

                    i = rowData.index('TP') + 1
                    tpVal = float(rowData[i])

                    ld = np.append(ld, [[rpmVal,tpVal,afrVal]],0)

    # remove first value (zeros), then pass data to caller
    ld = np.delete(ld,0, 0)                
    
    return ld

if __name__ == "__main__":
    logData = readLogData(logFilePath + "\\" + logFileFile)
    print('Number of valid data samples found:')
    print(logData.shape[0])
    extractLogVals(tps=10.0, rpm=3500.0, ld=logData)

