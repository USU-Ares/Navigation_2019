'''
Kaden Archibald
USU Ares Team

Created: Jan 12, 2019
Revised: Feb 7, 2019
Version: IPython 6.2.1 (Anaconda distribution) with Python 3.6.4

Main GUI Auxillary Functions 
(Mainly Used for Testing and May Not be Used During Actual Operation)
'''

import random as rand
import time

# The file name may need to be specified with a working directory
path = '~/usr/src/'
fileName = 'positionData.txt'

def getLatestPositionData():
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
        
        # Ensure that there is data in the target file
        errorMsg = 'There is no data to be read from ' + fileName
        assert len(rawData), errorMsg
        
        # Find the last line, split the two numbers apart, and convert to float
        xyPos = [float(num) for num in rawData[len(rawData)-1].split()]
    
    return xyPos


def writeRand():
    '''
    Testing function to generate random data to be plotted to simulate
    rover position. 
    '''
    
    with open(fileName, 'w') as outFile:
        # Open file in 'w' mode, which will erase any old data and
        # write new data 
        
        sleepTime = 0.1
        
        while True:
            outFile.write(str(rand.random()) + ' ' + str(rand.random()) + '\n')
            outFile.flush()
            time.sleep(sleepTime)

    return None


if __name__ == '__main__':
    
    try:
        # Simulate rover position data
        print('Writing Data...')
        writeRand()
        
    except KeyboardInterrupt:
        print('Terminating')
        
    finally:
        # Clear the data from this file
        with open(fileName, 'w'):
            pass