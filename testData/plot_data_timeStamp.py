import numpy as np
import matplotlib.pyplot as plt

fnames = ['passiveTest.log', 'passiveTest2.log', 'passiveTest3.log']
readings = []

def file_len(fname):
    with open(fname) as f:
        for i, l in enumerate(f):
            pass
    return i+1

"""
initialize arrays to hold the data
each row is [type, time, yaw]
type is 0 for chair, 1 for beacon
time is in seconds
yaw is in degrees [-180, 180]
"""
"""
for i in range(len(fnames)):
    nLines = file_len(fnames[i])
    readings[i] = np.empty([nLines,3])
    readings[i][:] = np.NAN
"""

def parse_data(fname):
    nLines = file_len(fname)
    dataArr = np.empty((nLines,3))
    dataArr[:] = np.NAN

    with open(fname) as f:
        for idx, line in enumerate(f):
            f_parts = line.split() 
            if len(f_parts) != 4:
                continue
           
            try:
                if f_parts[0] == 'chair':
                    f_parts
                    dataArr[idx,0] = 0
                    dataArr[idx,1] = makeTime(int(f_parts[1]), int(f_parts[2]))
                    dataArr[idx,2] = float(f_parts[3])
                elif f_parts[0]  == 'beacon':
                    dataArr[idx,0] = 1
                    dataArr[idx,1] = makeTime(int(f_parts[1]), int(f_parts[2]))
                    dataArr[idx,2] = float(f_parts[3])
                else:
                    continue
            except ValueError:
                print "Corrupted print out at line ", idx
    
    # remove rows with nan values
    dataArr = dataArr[~np.isnan(dataArr).any(axis=1)]
    return dataArr

def makeTime(secs, nsecs):
# convert sec and nsec to seconds as a float
    return secs + nsecs*1.0/1e9

# Parse data
for i in range(len(fnames)):
    readings.append(parse_data(fnames[i]))


def getSinCos(data):
    return np.sin(np.deg2rad(data)), np.cos(np.deg2rad(data))

def angleWrap(theta):
    if theta > 180:
        return theta-360
    elif theta < -180:
        return theta+360
    else:
        return theta
wrapFun = np.vectorize(angleWrap)

def makeSameT(chairData, beaconData):
    # always return chairData, beaconData
    if chairData.shape[0] < beaconData.shape[0]:
        # interpolate chairData
        #import ipdb; ipdb.set_trace()
        x = beaconData[:,0]
        newData = np.zeros((len(x), 2))
        newData[:,0] = x 
        newData[:,1] = np.interp(x, chairData[:,0], chairData[:,1])
        return newData, beaconData
    elif chairData.shape[0] > beaconData.shape[0]:
        # interpolate beaconData
        x = chairData[:,0]
        newData = np.zeros((len(x), 2))
        newData[:,0] = x 
        newData[:,1] = np.interp(x, beaconData[:,0], beaconData[:,1])
        return chairData, newData

def plotData(chairData, beaconData, offset):
    plt.figure()
    plt.plot(chairData[:,0], chairData[:,1], 'b')
    newData = wrapFun(-beaconData[:,1]+offset)
    plt.plot(beaconData[:,0], newData, 'r')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw degree')
    plt.show(block=False)
    return chairData[:,1], newData

def getBestOffset(chairData, beaconData):
    lo = 0.0
    hi = 120.0
    error = np.inf
    bestOffset = lo
    curOffset = lo
    
    while curOffset < hi:
        newData = wrapFun(-beaconData[:,1] + curOffset)
        mse = ((newData-chairData[:,1])**2).mean()
        #mse = np.mean((newData - chairData[:,1])**2)
        print "offset=", curOffset, ", mse=", mse, ", error=", error
        if mse < error:
            bestOffset = curOffset
            error = mse
        curOffset += 1

    print "Best offset is: ", bestOffset
    print "error is: ", error
    return bestOffset


"""
dataArr = readings[0]
chairData = dataArr[dataArr[:,0]==0, 1:]
beaconData = dataArr[dataArr[:,0]==1, 1:]
chairData, beaconData = makeSameT(chairData, beaconData)
"""

# find angle off-set by minimizing mean-squared error
offset = []
for i in range(len(fnames)):
    dataArr = readings[i]
    chairData = dataArr[dataArr[:,0]==0, 1:]
    beaconData = dataArr[dataArr[:,0]==1, 1:]
    chairData, beaconData = makeSameT(chairData, beaconData)
    offset.append(getBestOffset(chairData, beaconData))
    #cData, newData = plotData(chairData, beaconData, offset[i])
    cData, newData = plotData(chairData, beaconData, 60)

    plt.figure()
    plt.plot(cData-newData)
    plt.ylim([-30,30])
    plt.show(block=False)
'''
for i in range(len(fnames)):
    dataArr = readings[0]
    chairData = dataArr[dataArr[:,0]==0, 1:]
    beaconData = dataArr[dataArr[:,0]==1, 1:]
    chairData, beaconData = makeSameT(chairData, beaconData)
    plotData(chairData, beaconData, 70)
'''
