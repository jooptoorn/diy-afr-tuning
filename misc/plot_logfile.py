import re
import numpy as np
import matplotlib.pyplot as plt
# setting grid on here is easiest
plt.rcParams['axes.grid'] = True

logFilePath = "C:\\Users\\joopt\\OneDrive\\motor\\ride_home.log"
# logFilePath = "C:\\Users\\joopt\\OneDrive\\motor\\1st-ride-w-max-rev.log"

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

def plotAfrData(ld, iStart, iStop):
    # pd = plot data
    pdIdx = range(iStart,iStop)
    pdRpm = ld[iStart:iStop, 0]
    pdTp  = ld[iStart:iStop, 1]
    pdAfr = ld[iStart:iStop, 2]
    
    # create a figure with 2 y-axis and shared x for throttle and RPM
    # then create second figure for AFR
    # example https://matplotlib.org/3.4.3/gallery/ticks_and_spines/multiple_yaxis_with_spines.html
    fig, ax = plt.subplots(2, 1)
    twin1 = ax[0].twinx()
    fig.subplots_adjust(right=0.75)

    p1, = ax[0].plot(pdIdx,pdRpm, "b-", label="RPM")
    p2, = twin1.plot(pdIdx,pdTp,  "g-", label="Throttle open percentage")
    p3, = ax[1].plot(pdIdx,pdAfr, "r-", label="AFR")

    # constrain x to the values so the graphs look better
    ax[0].set_xlim(iStart, iStop)
    ax[1].set_xlim(iStart, iStop)

    # throttle visibility better when capped above 100%
    ax[0].set_ylim(0, 11000)

    # max rpm of rotax v990 is 10.500
    twin1.set_ylim(0, 110)

    # expecting AFR values roughly between 11 and 15
    ax[1].set_ylim(10, 16)

    ax[0].set_xlabel("Index")
    ax[0].set_ylabel("RPM")
    twin1.set_ylabel("Throttle open percentage")
    ax[1].set_xlabel("Index")
    ax[1].set_ylabel("Air-Fuel Ratio")
    
    ax[0].yaxis.label.set_color(p1.get_color())
    twin1.yaxis.label.set_color(p2.get_color())
    ax[1].yaxis.label.set_color(p3.get_color())

    tkw = dict(size=4, width=1.5)
    ax[0].tick_params(axis='y', colors=p1.get_color(), **tkw)
    twin1.tick_params(axis='y', colors=p2.get_color(), **tkw)
    ax[1].tick_params(axis='y', colors=p3.get_color(), **tkw)
    ax[0].tick_params(axis='x', **tkw)
    ax[1].tick_params(axis='x', **tkw)

    # no legend for now
    # ax[0].legend(handles=[p1, p2])
    # ax[1].legend(handles=[p3])
    plt.show()

if __name__ == "__main__":
    logData = readLogData(logFilePath)
    print('Number of valid data samples found:')
    print(logData.shape[0])
    # plotAfrData(logData, 0,logData.shape[0]-1)
    plotAfrData(logData, 5400,6000)
    # plotAfrData(logData, 2500,5000)