
import PathPlanner



def main():
    
    GPSdata = [0, 0, 7.19457e-5, 7.19457e-5]
    
    temp = PathPlanner.PathPlanner(*GPSdata)
    print(temp.planPath(*GPSdata))
    
    vector<vector<double>> gradientMap = {
        {9, 8, 9, 7, 8, 9, 1, 1},
        {9, 8, 9, 7, 9, 8, 1, 7},
        {7, 7, 9, 9, 7, 1, 1, 7},
        {9, 8, 7, 7, 9, 1, 9, 8},
        {9, 7, 1, 1, 1, 1, 9, 7},
        {8, 9, 1, 8, 7, 8, 9, 7},
        {7, 1, 1, 9, 8, 9, 7, 9},
        {1, 1, 9, 7, 7, 9, 8, 8}
    };
    
    
    return None
main()