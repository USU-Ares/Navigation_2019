
import PathPlanner

def loadFromFile(filename):
    file = open(filename,'r')
    readed = file.read().split("\n")
    file.close()

    # Get GPS data
    GPSdata = []
    temp = readed[0].split(" ")
    GPSdata.append(int(float(temp[0])))
    GPSdata.append(int(float(temp[1])))
    temp = readed[1].split(" ")
    GPSdata.append(int(float(temp[0])))
    GPSdata.append(int(float(temp[1])))

    # Get array
    costs = []
    for i in range(2,len(readed)-1):
        #print(readed[i].split())
        costs.append( [int(x) for x in readed[i].split()] )

    return (GPSdata, costs)

def main():
    
    """GPSdata = [0, 0, 7.19457e-5, 7.19457e-5]
    
    costs = [ [1, 1, 9, 7, 7, 9, 8, 8, 9],
              [7, 1, 1, 9, 8, 9, 7, 9, 9],
              [8, 9, 1, 8, 7, 8, 9, 7, 9],
              [9, 7, 1, 1, 1, 1, 9, 7, 9],
              [9, 8, 7, 7, 9, 1, 9, 8, 9],
              [7, 7, 9, 9, 7, 1, 1, 7, 9],
              [9, 8, 9, 7, 9, 1, 1, 7, 9],
              [9, 8, 9, 7, 8, 1, 7, 7, 7],
              [9, 8, 9, 7, 8, 1, 1, 1, 1] ]
              """
    (GPSdata, costs) = loadFromFile("testCostMap.txt")

    temp = PathPlanner.PathPlanner(*GPSdata)

    temp.updateCostMap(costs)

    # Display graph
    print("   ",end="")
    for i in range(len(costs)):
        print(i,end="  ")
    print()
    for i in range(len(costs)):
        print(str(i) + "" , costs[i])

    # Find graph
    print(temp.planPath(*GPSdata))

    
    return None

main()
