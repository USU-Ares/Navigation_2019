
import PathPlanner



def main():
    
    GPSdata = [0, 0, 7.19457e-5, 7.19457e-5]
    
    temp = PathPlanner.PathPlanner(*GPSdata)
    costs = [ [1, 1, 9, 7, 7, 9, 8, 8, 9],
              [7, 1, 1, 9, 8, 9, 7, 9, 9],
              [8, 9, 1, 8, 7, 8, 9, 7, 9],
              [9, 7, 1, 1, 1, 1, 9, 7, 9],
              [9, 8, 7, 7, 9, 1, 9, 8, 9],
              [7, 7, 9, 9, 7, 1, 1, 7, 9],
              [9, 8, 9, 7, 9, 1, 1, 7, 9],
              [9, 8, 9, 7, 8, 1, 7, 7, 7],
              [9, 8, 9, 7, 8, 1, 1, 1, 1] ]

    temp.updateCostMap(costs)

    print("   ",end="")
    for i in range(len(costs)):
        print(i,end="  ")
    print()
    for i in range(len(costs)):
        print(str(i) + "" , costs[i])
    print(temp.planPath(*GPSdata))

    
    return None

main()
