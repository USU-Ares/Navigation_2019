/*
Original Authors:
  Nicholas Wallace
  Kaden Archibald
  Brandon Deakin

ARES Team - Navigation & Autonomy
https://github.com/USU-Ares/Navigation_2019

Utah State University
College of Engineering

Description: Implementation of the A* Search algorithm for autonomous path planning.
Input: The initial and final GPS coordinates formatted as GPS structs (a latitude and longitude in decimal degrees)
       and a cost map formatted as a 2D array.
Output: The ideal trajectory for the path as a series of xy-coordinate waypoints.
*/

#include "path_planner.hpp"

#include <iostream>
#include <algorithm>
#include <vector>

#include <unistd.h>

/*
** Notes
**
** Still need to integrate this with ROS. rawCostMap will be found with a ROS subscriber node.
** Check the equality operator for Location struct.
** Check why the GPS coordinates change when the goal GPS is loaded into a Location struct.
**   Original goal coor is 1.25568935e-6 rad, but when loaded into a Location object the coor is changed
**   to 1.917642e-8.
*/

PathPlanner::PathPlanner(GPS start, GPS goal, std::vector<std::vector<double>> rawCostMap) :
    m_start(start),
    m_goal(goal),
    m_currentGoal(goal)
{
    // Initialize the start and goal member variables

    // Set min and max GPS values
    m_min.lat = (start.lat < goal.lat) ? start.lat : goal.lat ;
    m_min.lon = (start.lon < goal.lon) ? start.lon : goal.lon ;
    m_max.lat = (start.lat > goal.lat) ? start.lat : goal.lat ;
    m_max.lon = (start.lon > goal.lon) ? start.lon : goal.lon ;

    // Initialize the raw data for the cost map, which is the 2d field of gradients. This will
    // be converted into an actual cost map of Location objects below.
    setCostMap(rawCostMap);

    std::cout << "Created PathPlanner instance\n";
}
/*
PathPlanner::PathPlanner(GPS min, GPS max, GPS start, GPS goal) {
    m_min = min;
    m_max = max;
    m_start = start;
}
*/

PathPlanner::~PathPlanner() {
    //std::cout << "Destructing object\n";
    /*if (m_costMap != nullptr) {
        delete m_costMap;
        m_costMap = nullptr;
    }*/
}

double PathPlanner::get_gradientScore(Location current) {
    // Get the cost value from the current cell

    return current.getGradientScore();
}

// Estimate remaining cost
double PathPlanner::get_heuristicScore(Location current) {
    // Get distance between current Location and m_goal, returning that distance
    return current.getHeuristicScore();
}

/**
 * Return TRUE if heuristicScore is lower than previous hScore and was updated
 */
bool PathPlanner::calculate_totalScore(Location &current, Location &neighbor) {
    // Get partial scores
    double newGradient  = calculate_gradientScore(current, neighbor);
    double newHeuristic = calculate_heuristicScore(current);
    double newTotal = newGradient + newHeuristic;

    // If total score is lower, update
    if (newTotal < current.getTotalScore()) {
        current.setHeuristicScore(newHeuristic);
        current.setGradientScore(newGradient);
        current.setTotalScore();
        return true;
    }
    return false;
}
bool PathPlanner::calculate_gradientScore(Location &target, Location &current) {
    double oldGradient = target.getGradientScore();
    double newGradient = current.getGradientScore() + target.getCost();
    if (newGradient < oldGradient) {
        current.setGradientScore( newGradient );
        target.setPrev(&current);
        return true;
    }
    return false;
}
bool PathPlanner::calculate_heuristicScore(Location &current) {
    double oldHeuristic = current.getHeuristicScore();
    double newHeuristic = taxicab(current, m_currentGoal);
    if (newHeuristic < oldHeuristic) {
        current.setHeuristicScore( newHeuristic );
        return true;
    }
    return false;
}


std::vector<std::vector<int>> PathPlanner::planPath(GPS currentGPS, GPS goal) {
    // Get current coordinate on the grid
    Location startLocation = getLocation(currentGPS);
    Location goalNode      = getLocation(goal);

    std::cout << "***** current GPS\n";
    currentGPS.print();
    std::cout << "***** goal GPS\n";
    goal.print();
    std::cout << "***** Current node\n";
    startLocation.print();
    std::cout << "***** Goal node\n";
    goalNode.print();

    // Sets of nodes that have been visited or to be checked
    std::vector<Location> closedSet;
    std::vector<Location> openSet;

    // Inital condition
    startLocation.setGradientScore(0);
    startLocation.setTotalScore(0);
    openSet.push_back(startLocation);

    // Loop until our open set is empty
    while (openSet.size() > 0) {
        std::cout << "About to sleep\n";
        //sleep(1);
        std::cout << "Open set Size: " << openSet.size() << "\n";
        for (int i=0; i<openSet.size(); i++) {
            std::cout << "  ";
            openSet[i].print();
        }
        
        std::cout << "Closed set size: " << closedSet.size() << "\n";
        for (int i=0; i<closedSet.size(); i++) {
            std::cout << "  ";
            closedSet[i].print();
        }

        // Get lowest cost item from openSet
        Location currentLocation = getMin(openSet);
        std::cout << "Min from open: ";
        currentLocation.print();
        std::cout << "Returned value: "; getMin(openSet);
        std::cout << "\n";
        // Check if we have reached the goal

        if (currentLocation == goalNode) {

            std::cout << "current was equal to goalNode\n";
            currentLocation.print();
            goalNode.print();
            std::cout << "&&&&&&\n";
            // Complete March 9, not tested
            // Reconstruct path

            std::vector<std::vector<int>> endPath;

            Location* trackingPtr = &currentLocation;
            while (trackingPtr != nullptr)
            {
                // Get current location position
                int x, y;
                //std::cout << "Calling getBoardIndex\n";
                getBoardIndex(trackingPtr, &x, &y);

                // Add location to path vector
                std::vector<int> pathPos {y,x};
                endPath.push_back(pathPos);

                // Move to the next location
                trackingPtr = trackingPtr->getPrev();
            }

            // Return the path
            std::cout << "I'm a disapointment of a function!" << std::endl;
            return endPath;
        }
        std::cout << "was not the goal node\n";

        // Remove current from openSet, and add to closedSet
        std::cout << "*************\n";
        std::cout << "Calling remove min\n";
        //openSet = removeMin(openSet); // TODO Change openSet to be a min heap
        removeMin(openSet);
        std::cout << "done with remove min\tSize: " << openSet.size() << "\n";
        for (int i=0; i<openSet.size(); i++) {
            std::cout << "  ";
            openSet[i].print();
        }
        std::cout << "*************\n";

        closedSet.push_back(currentLocation);

        std::cout << "\t\tCurrentLocation after delete\n";
        currentLocation.print();
        std::cout << "\t\t*******\n";
        // Check each neighbor of currentLocation
        std::vector<Location> neighbors = getNeighbors(currentLocation);
        std::cout << "Neighbors of current: " << neighbors.size() << "\n";
        for (int i=0; i<neighbors.size(); i++) {
            std::cout << "  ";
            neighbors[i].print();
        }

        for (int i=0; i<neighbors.size(); i++) {
            // Check if neighbor is not in the closed set, and analyze the nodes that are not.
            if (!inSet(closedSet, neighbors[i]))
            {
                std::cout << "not in closed set, so doing the loop\n"; neighbors[i].print();
                // Distance from start to neighbor
                double temp_gradientScore = currentLocation.getGradientScore() + neighbors[i].getCost();
                std::cout << "temp_gradientScore: " << temp_gradientScore << "\n";

                // If our neighbor is not in the openSet, push to openSet
                if (!inSet(openSet, neighbors[i])) {
                    // Update scores
                    neighbors[i].setGradientScore(temp_gradientScore);
                    neighbors[i].setHeuristicScore(calculate_heuristicScore(neighbors[i]));
                    calculate_totalScore(currentLocation, neighbors[i]);
                    // Push neighbor to openSet
                    openSet.push_back(neighbors[i]);

                    std::cout << "Was not in OpenSet\n";
                }
                // If, our neighbor is in the openSet, check if it is a better gradient score
                // if the gradient score is better, remove old Node and push new neighbor
                else {
                    // Linear probe to find previous occurance of neighbor
                    for (int j=0; j<openSet.size(); j++) {
                        // We found it!
                        if (openSet[j] == neighbors[i] && openSet[j] < neighbors[i]) {
                            openSet.erase(openSet.begin() + j);
                            // Insert new neighbor into openSet

                            // Update scores
                            neighbors[i].setGradientScore(temp_gradientScore);
                            neighbors[i].setHeuristicScore(calculate_heuristicScore(neighbors[i]));
                            calculate_totalScore(currentLocation, neighbors[i]);
                            // Push neighbor to openSet
                            openSet.push_back(neighbors[i]);

                            std::cout << "already in OpenSet\n";
                            break;
                        }
                    }
                }

                std::cout << "Done with this iteration of neighbor\n";
            }
        } // End neighbor checking
        std::cout << "Done with neighbor checking\n";
    }
    std::cout << "How did you get here?\n";
}

// Row major
void PathPlanner::setCostMap(std::vector<std::vector<double>> rawCostMap) {
    // Loop through raw cost map, and create final cost map using getBoardIndex
    double delta_lat = (m_max.lat-m_min.lat) / (rawCostMap.size()-1);
    double delta_lon = (m_max.lon-m_min.lon) / (rawCostMap[0].size()-1);

    //std::cout << "Delta latitute: " << delta_lat << "\n";
    //std::cout << "Delta longitude: " << delta_lon << "\n";

    /*
    std::cout << "Raw cost map[0] size: " << rawCostMap[0].size() << std::endl;
    std::cout << "Raw cost map    size: " << rawCostMap.size() << std::endl;
    std::cout << "max.lat-min.lat: " << (m_max.lat-m_min.lat) << "\n";
    std::cout << "max.lon-min.lon: " << (m_max.lon-m_min.lon) << "\n";
    std::cout << "Delta lat: " << delta_lat << "\n";
    std::cout << "Delta lon: " << delta_lon << "\n";
    */

    m_costMap = std::vector<std::vector<Location>>();

    for (int y=0; y<rawCostMap.size(); y++) {
        std::vector<Location> locationRow = std::vector<Location>();
        for (int x=0; x<rawCostMap[y].size(); x++) {
            //std::cout << "i, j: " << i << " " << j << std::endl;
            //locationRow.push_back(Location(rawCostMap[y][x], delta_lat*y, delta_lon*x));
            locationRow.push_back( Location( GPS(m_min.lat + delta_lat*y, m_min.lon + delta_lon*x, false), rawCostMap[y][x] ) );
        }
        m_costMap.push_back(locationRow);
    }
}

Location& PathPlanner::getLocation(GPS point) {
    int x = -1,
        y = -1;
    std::cout<<"AAA\n";
    point.print();
    getBoardIndex(&point, &x, &y);

    std::cout << "X: " << x << "\tY: " << y << "\n";
    return m_costMap[x][y];
}
void PathPlanner::getBoardIndex(const GPS* loc, int* p_x, int* p_y) {
    // This isn't working, but should be used for constant lookup time.
    // Commenting out to prove that rest of algorithm works, and replacing with linear search
    /*
    // Get proportion of GPS latitude between min and max latitude
    double lat_proportion = ((double)(loc->lat - m_min.lat))/(m_max.lat - m_min.lat);
    // Get proportion of GPS longitude between min and max longitude
    double lon_proportion = ((double)(loc->lon - m_min.lon))/(m_max.lon - m_min.lon);

    // Set x from latitute proportion
    *p_x = (int)(lon_proportion * (m_costMap[0].size()) );
    // Set y from longitude proportion
    *p_y = (int)(lat_proportion * (m_costMap.size()) );

#if 1
    std::cout << "Get board index debug\n";
    std::cout << "loc: " << loc->lat << "\t" << loc->lon << "\n";
    std::cout << "min: " << m_min.lat << "\t" << m_min.lon << "\n";
    std::cout << "max: " << m_max.lat << "\t" << m_max.lon << "\n";
    std::cout << "loc->lat   - m_min.lat: " << (loc->lat - m_min.lat) << "\n";
    std::cout << "m_max->lat - m_min.lat: " << (m_max.lat - m_min.lat) << "\n";
    std::cout << "Lat prop: " << lat_proportion << "\n";
    std::cout << "Lon prop: " << lon_proportion << "\n";
    std::cout << "m_costMap[0].size()-1: " << m_costMap[0].size() << "\n";
    std::cout << "m_costMap.size()-1: " << m_costMap.size() << "\n";
#endif
    */
    for (int i=0; i<m_costMap.size(); i++) {
        for (int j=0; j<m_costMap[i].size(); j++) {
            //std::cout << "I: " << i << "\tJ: " << j << "\tlat: " << m_costMap[i][j].lat << "\tlon: " << m_costMap[i][j].lon << "\n";
            if (m_costMap[i][j] == *loc) {
                *p_y = i;
                *p_x = j;
                return;
            }
        }
    }
}

/*
void PathPlanner::updatePartOfCostMap(std::vector<std::vector<double>> &costMap, int x_offset, int y_offset) {
    for (int i=0; i<costMap.size(); i++) {
        for (int j=0; j<costMap[i].size(); j++) {
            m_costMap[i+x_offset][j+y_offset] = costMap[i][j];
        }
    }
}
*/
Location getMin(std::vector<Location> &vec) {
    // Find the Location with the lowest total score from the open set

	Location min = vec[0];
	for (int i = 0; i < vec.size(); i++)
	{
		if (vec[i].getTotalScore() < min.getTotalScore())
		{
			min = vec[i];
		}
	}

    std::cout << "getMin: ";
    min.print();

    return min;
}
//std::vector<Location> PathPlanner::removeMin(std::vector<Location> vec) {
void removeMin(std::vector<Location> &vec) {
    // Find and remove the Location with minimum f score from the open set

	int minIndex = 0;
	for (int i = 1; i < vec.size(); i++)
	{
        // Standard linear scan
		if (vec[i].getTotalScore() < vec[minIndex].getTotalScore())
		{
			minIndex = i;
		}
	}

    //std::cout << "removeMin index: " << minIndex << "\tsize before erase: " << set.size() << "\n";
    vec.erase(vec.begin() + minIndex);
    //std::cout << "size after erase: " << set.size() << "\n";
    //return vec;
}

bool inSet(std::vector<Location> &set, Location& entry) {
    for (int i=0; i<set.size(); i++) {
        if (set[i] == entry) {
            /*
            std::cout << "####\n";
            entry.print();
            std::cout << "FOUND ENTRY\n";
            std::cout << "####\n";
            */
            return true;
        }
    }
    return false;
    //return std::find(set.begin(), set.end(), entry) != set.end();
}

std::vector<Location> PathPlanner::getNeighbors(Location &node) {
    // Return the north, east, south and west neighbors of the current Location node

    // Completed March 17, not yet tested
	std::vector<Location> neighbors;

    // Assume a 2D array and access the four neighbors. Assume that the cost map will contain
    // location objects, and that these location objects are located in the array according
    // to there x and y member variables.
    int x = -1;
    int y = -1;
    std::cout<<"BBB\n";
    node.print();
    getBoardIndex(&node, &x, &y);
    std::cout << "getNeighbors of X: " << x << "\tY: " << y << "\n";
    if (y < m_costMap.size()-1) {
        neighbors.push_back(m_costMap[y+1][x  ]);
    }
    if (x < m_costMap[0].size()-1) {
        neighbors.push_back(m_costMap[y  ][x+1]);
    }
    if (y > 0) {
        neighbors.push_back(m_costMap[y-1][x  ]);
    }
    if (x > 0) {
        neighbors.push_back(m_costMap[y  ][x-1]);
    }

    return neighbors;
}

double PathPlanner::taxicab(GPS start, GPS end) {
    int x_1, y_1;
    int x_2, y_2;
    //std::cout<<"CCC\n";
    getBoardIndex(&start, &x_1, &y_1);
    //std::cout<<"DDD\n";
    getBoardIndex(&end  , &x_2, &y_2);
    // Get taxicab distance between two locations
    unsigned deltaX = abs(x_1 - x_2);
    unsigned deltaY = abs(y_1 - y_2);

    // Return taxicab distance
    return deltaX + deltaY;
}
