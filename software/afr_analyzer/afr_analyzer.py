import tkinter as tk
import re
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

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
ftRpmVals = [x * 1.0 for x in range(500, 10500+1, 500)]

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
    
    return exstrLdRows

# calculates statistical properties of AFR values in dataset
# data has to have same format as logdata, i.e. [rpm, tps, afr]
def calcAfrStat(data):
    mu, std = norm.fit(data[:,2])
    return [mu, std]

# calculates AFR [average, std dev] for the entire fuel table RPM and TPS value range
# output data format: 
# [[rpm=500,   tps=0, [dataset], [mu, std], [rpm=500,   tps=2, [dataset], [mu, std], .. , [rpm=500,   tps=100, [dataset], [mu, std]]
# [[rpm=1000,  tps=0, [dataset], [mu, std], [rpm=1000,  tps=2, [dataset], [mu, std], .. , [rpm=1000,  tps=100, [dataset], [mu, std]]
# ..
# [[rpm=10500, tps=0, [dataset], [mu, std], [rpm=10500, tps=2, [dataset], [mu, std], .. , [rpm=10500, tps=100, [dataset], [mu, std]]

def calcAfrTable(ld):
    # works by looping all colums then rows of the fuel table,
    # and calculating AFR statistics for all cells
    ftAfrStat = []

    for ftTp in ftTpVals:
        ftAfrStatColumn = [[]]
        for ftRpm in ftRpmVals:
            afrData = extractLogVals(ftTp, ftRpm,ld)
            afrStat = calcAfrStat(afrData)
            ftAfrCell = [ftRpm, ftTp, afrData, afrStat]
            if(ftAfrStatColumn==[[]]):
                ftAfrStatColumn = [ftAfrCell]
            else:
                ftAfrStatColumn.append(ftAfrCell)
            # print(len(ftAfrStatColumn))
            # print(len(ftAfrStatColumn[0][0]))

        # at this point we have the entire column, append to the rest of the AFR data table
        if(ftAfrStat==[]):
            ftAfrStat = [ftAfrStatColumn]
        else:
            ftAfrStat.append(ftAfrStatColumn)
    
    return ftAfrStat

# print the AFR-statistic table 
def printAfrTable(ast):
    # top row
    topStr = "RPM-TPS\t"
    for tp in ftTpVals:
        topStr = topStr + str(tp) + '\t'
    print(topStr)

    for i in range(0,len(ftRpmVals)):
        rowStr = str(ftRpmVals[i]) + '\t'
        for j in range(0,len(ftTpVals)):
            cell = ast[j][i]
            afr = cell[3][0]
            std = cell[3][1]
            afrText = "{:.1f}".format(afr)
            rowStr = rowStr + afrText + '\t'
        print(rowStr)
            

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
    # eVals = extractLogVals(tps=5.0, rpm=4000.0, ld=logData)
    # stats = calcAfrStat(eVals)
    # print(stats)
    ast = calcAfrTable(logData)
    printAfrTable(ast)
    

