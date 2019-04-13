'''
Original Author: Kaden Archibald
ARES Team - Navigation & Autonomy
https://github.com/USU-Ares/Navigation_2019

Utah State University
College of Engineering

Created: April 1, 2019
Revised: April 7, 2019
Version: IPython 6.2.1 (Anaconda distribution) with Python 3.6.4

Test Cases for A* Search Module
'''

from random import randint, seed
seed(0)
from math import pi



# Constants in SI units
earthRadius = 6378e3


# Utility functions
def degToRad(angle): 
    return angle * (pi/180)

def radToDeg(angle):
    return angle * (180/pi)


# Generate start/stop GPS points, a cost map (format as matrix of positive 
# integers), write data to file.
latSize = 25
lonSize = 25
nodeSize = 1 # meters
maxGradient = 99

costMapFile = 'testCostMap.txt'


def main(*args, **kwargs):
    
    # Input in deg
    startGPS = (40, -111) 
    startGPS = tuple(map(degToRad, startGPS))
    
    
    # Find goal GPS point. Assume that the bottom lef corner is the start 
    # and the top right corner is the goal.
    xShift = lonSize * nodeSize
    yShift = latSize * nodeSize
    latShift = yShift / earthRadius
    lonShift = xShift / earthRadius
    endGPS = (startGPS[0] + latShift, startGPS[1] + lonShift)
    
    
    # Generate cost map, assuming a square map for now 
    costMap = [[randint(10, maxGradient) for j in range(lonSize)] for \
                i in range(latSize)]
    
    
    
    # Write information to file
    with open(costMapFile, 'w') as outFile:  
        # GPS
        # Format as:
        # StartLat StartLon
        # EndLat EndLon
        out = ''
        out += str(radToDeg(startGPS[0])) +  ' ' + \
            str(radToDeg(startGPS[1])) + '\n'
        out += str(radToDeg(endGPS[0])) + ' ' + \
            str(radToDeg(endGPS[1])) + '\n'
        outFile.write(out)
        
        # Cost Map
        for row in costMap:
            out = ''
            for cost in row:
                out += str(cost) + ' '
            
            outFile.write(out + '\n')
        

    debug = True
    if debug:
        print('Start GPS in Radians:    ', startGPS)
        print('End GPS in Radians:      ', endGPS)
    
    return None



if __name__ == '__main__':
    main()    

        
        
    