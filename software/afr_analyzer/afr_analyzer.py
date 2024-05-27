import tkinter as tk
import re
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

# setting grid on here is easiest
plt.rcParams['axes.grid'] = True

logFilePath = "C:\\Users\\joopt\\OneDrive\\motor\\rsv mille\\tuning\\logger\\testing\\20240520 first ride no accel pump"
# logFileFile = "warmup.log"
# logFileFile = "1st-ride-w-max-rev.log"
logFileFile = "ride_home.log"

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

# filtering parameters
# 
#
#       Throttle variation.
#
# Taking a window of 2*filTpWindowLen + 1 sample, calculate the average Tp value within the window. 
# Sum the magnitude of the delta of each sample with that average to obtain a measure of the variation 
# within the window, accepting (an average) maximum variation of filTpMaxSumDelta per sample
# Consider X samples before and after a sample to detect variations
filTpWindowLen = 2

# This parameter is the variation PER SAMPLE, during filtering it will be multiplied with window size to
# obtain a total maximum sum for the entire window
filTpMaxSumDelta = 3.0

# Throw away filTpFilterLen samples after end of the throttle variation to ignore 'after effects' on AFR of the variation
filTpFilterLen = 12

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
        rowStrU = str(ftRpmVals[i]) + '\t'
        rowStrD = '\t'
        for j in range(0,len(ftTpVals)):
            cell = ast[j][i]
            afr = cell[3][0]
            std = cell[3][1]
            afrText = "{:.1f}".format(afr)
            stdText = "{:.1f}".format(std)
            rowStrU = rowStrU + afrText + '\t'
            rowStrD = rowStrD + stdText + '\t'
        # print(rowStrU)
        print(rowStrD)

# print average standard deviation in AFR-statistics table. Used for debugging only
def printAfrStdAvg(ast):
    # loop over all elements and add any valid standard deviation (i.e. not NaN) to array
    stdSum = []
    for i in range(0,len(ftRpmVals)):
        for j in range(0,len(ftTpVals)):
            cell = ast[j][i]
            std = cell[3][1]
            if(not(np.isnan(std))):
                stdSum.append(std)
    stdAvg = sum(stdSum)/len(stdSum)
    print("Average standard deviation in AFR values: {:.2f}".format(stdAvg))

# remove logdata where the throttle has just been opened or closed
# throttle variations cause considerable AFR fluctuations and we do not
# want to base fuel map improvements on such values. 
# This particular filtering can only be done for 
# the original log where values are chronologically ordered
def filterThrottleVariation(ld):
    # check if logfile contains enough entries so we can analyse at least 1 window
    if(ld.shape[0] < 2*filTpWindowLen + 1):
        print("Error: not enough log entries for throttle variation filtering")
        return []

    # copy all logdata first, then start throwing away invalid portions of the log
    fld = ld
    
    # total sum-of-deltas in a filter window will be compared against maximum allowable delta
    # first need to scale the allowable delta-per-sample with window size
    maxSumDelta = filTpMaxSumDelta * (filTpWindowLen*2 + 1)

    # initialize the vector that will delete samples in fld
    filterAllIdx = []

    # Now scan for throttle data. For each variation, find the beginning and end
    # loops over the target samples, skipping first and last samples
    i = filTpWindowLen
    while i < ld.shape[0]-filTpWindowLen:
        # calculate average Tp value around the target
        avgTp = np.average(ld[i-filTpWindowLen:i+filTpWindowLen+1,1])

        # calculate sum-of-delta around the target sample
        deltaTpVec = np.abs(ld[i-filTpWindowLen:i+filTpWindowLen+1,1] - avgTp)
        deltaTpSum = np.sum(deltaTpVec)


        # is the sum larger than the allowed maximum?
        if(deltaTpSum > maxSumDelta):
            # This is the start of a variation. Find the end using the same arithmetic
            idxVarStart = i


            for j in range(i+1, ld.shape[0]-filTpWindowLen):
                # calculate average Tp value around the target
                avgTp = np.average(ld[j-filTpWindowLen:j+filTpWindowLen+1,1])

                # calculate sum-of-delta around the target sample
                deltaTpVec = np.abs(ld[j-filTpWindowLen:j+filTpWindowLen+1,1] - avgTp)
                deltaTpSum = np.sum(deltaTpVec)

                # is the sum smaller than the allowed maximum? We found the end of the variation
                if(deltaTpSum <= maxSumDelta):
                    break
            
            # j is now either at the end of the variation or at the end of the logdata
            idxVarEnd = j
            if(idxVarEnd + filTpFilterLen >= ld.shape[0]):
                # reached end of logdata. Filter out everything up to the last sample
                idxFilterEnd = ld.shape[0]-1
            else:
                # reached the end of the variation. Filter out all data up to X samples after the variation
                idxFilterEnd = idxVarEnd + filTpFilterLen

            # indexes of logdata to be removed for this variation
            # filterIdx = [idxVarStart, idxFilterEnd]

            # append to array of all variations
            for k in range(idxVarStart, idxFilterEnd + 1):
                filterAllIdx.append(k)

            # set i to new value to prevent multiple detections of same variation
            # note that we may detect a new variation within the section that will be cut out
            # as i is set to idxVarEnd rather than idxFilterEnd. This is done on purpose because
            # any variation will lead to more AFR fluctuations, and we should try to detect ALL variations
            # including in parts that will be deleted.
            i = idxVarEnd

        else: # no variation was found
            i = i+1
    
    # at this point all data was scanned. See if there are variations, and delete samples from logdata if so 
    if(filterAllIdx != []):
        # remove duplicate deletions
        filterAllIdx = np.unique(filterAllIdx)
        fld = np.delete(fld, filterAllIdx, 0)
    
    # print("total number of entries deleted: {:d}".format(tot))
    print("total number of entries deleted: {:d}".format(len(filterAllIdx)))

    return fld

#
def filterMedian(cell):
    fcell = cell
    return fcell

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

    filteredLogData = filterThrottleVariation(logData)
    # eVals = extractLogVals(tps=5.0, rpm=4000.0, ld=logData)
    # stats = calcAfrStat(eVals)
    # print(stats)
    ast = calcAfrTable(filteredLogData)
    
    printAfrStdAvg(ast)
    printAfrTable(ast)
    

