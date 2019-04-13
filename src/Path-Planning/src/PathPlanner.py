
from math import pi, sin, cos, asin
from math import sqrt, ceil, floor

from copy import deepcopy

earthRadius = 6378e3


class PathPlanner:
    def __init__(self, startLat, startLon, endLat, endLon):
        
        # Parameters
        self.startLat = self.degToRad(startLat)
        self.startLon = self.degToRad(startLon)
        self.endLat = self.degToRad(endLat)
        self.endLon = self.degToRad(endLon)
        
        # Data
        self.cellSize = 1
        
        self.deltaLat = abs(self.endLat - self.startLat)
        self.deltaLon = abs(self.endLon - self.startLon)
        
        deltaX = floor(2 * earthRadius * asin(sqrt( cos(startLat)**2 * sin(self.deltaLon/2)**2 ) ) )
        deltaY = floor(earthRadius * self.deltaLat)
        
        self.distance = sqrt(deltaX**2 + deltaY**2) # Flat Earth Model
        
        print("DeltaX: ",deltaX)
        print("DeltaY: ",deltaY)
        print("Cell size: ", self.cellSize)
        self.cellNumX = deltaX // self.cellSize
        self.cellNumY = deltaY // self.cellSize
        
        # Dictionary object will not support a deepcopy when assigned to its own variable.
        self.costMap = [[{'cost': 0, 'gradient': 0, 'heuristic': 0, 'total': 0, 'prev': None} \
                         for i in range(self.cellNumX+1)] for j in range(self.cellNumY+1)]
        print("cellNumX: ",self.cellNumX)
        print("cellNumY: ",self.cellNumY)
    
    
        
    def planPath(self, currentLat, currentLon, goalLat, goalLon):
        
        # Find current position
        currentPos = self.findCoor(currentLat, currentLon)
        goalPos = self.findCoor(goalLat, goalLon)
        
        
        # Initialize containers
        closedSet = []
        openSet = [currentPos]
        
        # Set scores to null
        for row in self.costMap:
            for element in row:
                element['gradient'] = None
                element['heuristic'] = None
        
        
        self.costMap[currentPos[0]][currentPos[1]]['gradient'] = 0
        self.costMap[currentPos[0]][currentPos[1]]['heuristic'] = self.calcHeuristic(currentPos, goalPos)

        while openSet:
            
            # Assign the node with the minimum gradient value to the current cell
            currentPos = self.extractMin(openSet)
            
            if currentPos == goalPos:
                return self.reconstructPath(goalPos)
            
            closedSet.append(currentPos)
            
            neighbors = self.findNeighbors(currentPos)
            
            for element in neighbors:
                if element not in closedSet:
                    tempGradient = self.costMap[currentPos[0]][currentPos[1]]['gradient']
                    tempGradient += self.costMap[element[0]][element[1]]['cost']
            
                    if not element in openSet:
                        openSet.append(element)
                    elif tempGradient >= self.costMap[element[0]][element[1]]['gradient']:
                        continue
                    
                    self.costMap[element[0]][element[1]]['prev'] = currentPos
                    self.costMap[element[0]][element[1]]['gradient'] = tempGradient
                    self.costMap[element[0]][element[1]]['heuristic'] = self.calcHeuristic(element, goalPos)
                    self.costMap[element[0]][element[1]]['total'] = tempGradient + self.calcHeuristic(element, goalPos)
    
    
    
    def calcHeuristic(self, currentPos, goalPos):
        dx = abs(goalPos[1] - currentPos[1])
        dy = abs(goalPos[0] - currentPos[1])
        
        return dx + dy
    
    
    def findNeighbors(self, node):
        
        neighbors = []
        if node[0] > 0:
            neighbors.append((node[0]-1, node[1]  ))
        if node[1] > 0:
            neighbors.append((node[0]  , node[1]-1))
        if node[0] < len(self.costMap)-1:
            neighbors.append((node[0]+1, node[1]  ))
        if node[1] < len(self.costMap[0])-1:
            neighbors.append((node[0]  , node[1]+1))
            
        return neighbors
        
    def reconstructPath(self, goalNode):
        
        currentNode = goalNode
        totalPath = []
        
        while not self.costMap[currentNode[0]][currentNode[1]]['prev'] is None:
            
            totalPath.append(currentNode)
            currentNode = self.costMap[currentNode[0]][currentNode[1]]['prev']
            
        return totalPath[::-1]
    
    
    def extractMin(self, array):
        
        minIndex = 0
        minItem = array[0]
        for i in range(len(array)):
            if self.costMap[array[i][0]][array[i][1]]['total'] < \
               self.costMap[minItem[0]][minItem[1]]['total']:
                minIndex = i
            minItem = deepcopy(array[minIndex])
        
        array.pop(minIndex)
        return minItem
        
    
    def findCoor(self, lat, lon):
        
        y = (self.degToRad(lat) - self.startLat) / self.deltaLat
        x = (self.degToRad(lon) - self.startLon) / self.deltaLon
        
        return (ceil(y * self.cellNumY), ceil(x * self.cellNumX))
        
    def updateCostMap(self, costs):
        print("Costs: " , len(costs), "\t", len(costs[0]))
        print("map: " , len(self.costMap), "\t", len(self.costMap[0]))
        for i in range(len(costs)):
            for j in range(len(costs[i])):
                self.costMap[i][j]['cost'] = costs[i][j]
        
    def degToRad(self, angle):
        print(type(angle))
        print("angle: ",angle,"\t",pi/180)
        print(angle * pi/180)
        return angle * (pi/180)
