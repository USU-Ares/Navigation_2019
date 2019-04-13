
import PathPlanner



def main():
    
    GPSdata = [0, 0, 7.19457e-5, 7.19457e-5]
    
    temp = PathPlanner.PathPlanner(*GPSdata)
    print(temp.planPath(*GPSdata))
    
    
    
    
    return None
main()