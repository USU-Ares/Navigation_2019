
from time import time

import PathPlanner
inputFile = 'testCostMap.txt'



def loadFromFile(filename):
    
    with open(filename, 'r') as file:
        readed = file.read().split("\n")
        readed.pop(len(readed)-1)
    

    # Get GPS data
    GPSdata = []
    temp = readed[0].split(" ")
    
    GPSdata.append(float(temp[0]))
    GPSdata.append(float(temp[1]))
    temp = readed[1].split(" ")
    
    GPSdata.append(float(temp[0]))
    GPSdata.append(float(temp[1]))
    
    

    # Get array
    costs = []
    for i in range(2,len(readed)-1):
        #print(readed[i].split())
        costs.append( [int(x) for x in readed[i].split()] )

    return (GPSdata, costs)



def testOne():
    
    print('TEST CASE ONE')
      
    GPSdata = [0, 0, 7.19457e-5, 7.19457e-5]
        
    costs = [ [1, 1, 9, 7, 7, 9, 8, 8, 9],
              [7, 1, 1, 9, 8, 9, 7, 9, 9],
              [8, 9, 1, 8, 7, 8, 9, 7, 9],
              [9, 7, 1, 1, 1, 1, 9, 7, 9],
              [9, 8, 7, 7, 9, 1, 9, 8, 9],
              [7, 7, 9, 9, 7, 1, 1, 7, 9],
              [9, 8, 9, 7, 9, 1, 1, 7, 9],
              [9, 8, 9, 7, 8, 1, 7, 7, 7],
              [9, 8, 9, 7, 8, 1, 1, 1, 1] ]
        
    temp = PathPlanner.PathPlanner(*GPSdata)
    temp.updateCostMap(costs)
    
    
    # Display graph
    print('\nCost Map...')
    print("   ",end="")
    for i in range(len(costs)):
        print(i,end="  ")
    print()
    for i in range(len(costs)):
        print(str(i) + "" , costs[i])

    # Find graph
    print('\nTrajectory...')
    out = temp.planPath(*GPSdata)
    string = ''
    
    
    for i in range(len(out)):
        string += str(out[i]) + '  '
        if not i % 8 and i > 0:
            string += '\n'
            
    print(string)

    
    return None


def testTwo():
    
    print('\n\n\n\nTEST CASE TWO')
    data = loadFromFile(inputFile)

    temp = PathPlanner.PathPlanner(*data[0])
    temp.updateCostMap(data[1])
    
    
    # Time plan path execution
    start = time()
    out = temp.planPath(*data[0])
    end = time()
    
    dt = end-start
    print('dt: ', dt)
    
    string = ''
    print('\nTrajectory...')
    for i in range(len(out)):
        string += str(out[i]) + '  '
        if not i % 8 and i > 0:
            string += '\n'
            
    print(string)






testCases = [testOne, testTwo]
for test in testCases:
    test()
            

    
    
    
    
    
    
    
    
    
    
    
    
    
