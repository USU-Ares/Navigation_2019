'''
Original Author: Kaden Archibald
ARES Team - Navigation & Autonomy

Utah State University
Department of Mechanical and Aerospace Engineering

Created: Jan 12, 2019
Revised: Feb 19, 2019
Version: IPython 6.2.1 (Anaconda distribution) with Python 3.6.4

Main GUI Auxillary Functions 
(Mainly Used for Testing Functionality when the ROS Network is not 
yet Developed and May Not be Used During Actual Operation)
'''


import random
import time

from math import pi, sin, cos

# The file name may need to be specified with a working directory
#path = '~/usr/src/'
fileName = 'positionData.txt'


def haversine(lat, lon):
    ''' Apply spherical geometry to convert latitude and longitude into 
    Cartesian coordinates. '''
    
    earthRadius = 6378e3  # meters
    
    xPos = earthRadius * lon
    yPos = earthRadius * lat
    
    return [xPos, yPos]


def degToRad(angle):
    return angle * (pi/180)


def getLatestPositionData(pos = 0):
    '''
    Get the position data from the rover sensors from a file. This function
    assumes that the data is space-seperated floating point numbers in 
    xy-coordinate pairs, one pair per line.
    
    input: None
    output: The x and y positions as read from a file. Both number are 
            type(float) and are stored in a list
    '''
    
    with open(fileName, 'r') as inFile:
        # Read in the data
        rawData = inFile.readlines()
        
        # Proceed if there is data in the target file, abort() otherwise
        errorMsg = 'There is no data to be read from ' + fileName
        assert len(rawData), errorMsg
        
        # Generate the data, split the two numbers apart, and convert to float
        xyPos = [float(num) for num in rawData[pos].split()]

    return xyPos


def writeTrialData():
    '''
    Testing function to generate random data to be plotted to simulate
    rover position. 
    '''
    
    with open(fileName, 'w') as outFile:
        # Open file in 'w' mode, which will erase any old data and
        # write new data 
        
        testNums = 1000
        sleepTime = 0.1
        
        i = 0
        while i < testNums:
            outFile.write(str(random.randint(i, i+3)) + ' ' + \
                          str(random.randint(i, i+3)) + '\n')
            
            outFile.flush()
            time.sleep(sleepTime)
            
            i += 1

    return None


if __name__ == '__main__':
    
    try:
        # Simulate rover position data
        print('Writing Data...')
        writeTrialData()
        
    except KeyboardInterrupt:
        print('Terminating')
        
    finally:
        pass
        # Clear the data from this file
#        with open(fileName, 'w'):
#            pass
        