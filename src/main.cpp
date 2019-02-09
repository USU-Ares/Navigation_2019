#include <iostream>
#include "path_planner.hpp"

int main() {
    GPS gps1 = GPS(1.0, 2.0);
    GPS gps2 = GPS(3.0, 2.5);

    std::cout << "GPS: " << gps1.lat << ", " << gps1.lon << std::endl;
    std::cout << "GPS: " << gps2.lat << ", " << gps2.lon << std::endl;
    
    std::cout << "The distance between the two points is: " << gps1-gps2 << std::endl;

    //PathPlanner planner = PathPlanner(gps1, gps2);
    return 0;
}
