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
** 1: A path planner instance will be created with the start and ending GPS coordinates,
** but a raw cost map will need to be sent in many times. Should the raw cost map be changed 
** so that it no longer is a parameter for the sonstructor? Is that what the second constructor
** is for?
**
** 2: Still need to integrate this with ROS. rawCostMap will be found with a ROS subscriber node. 
*/

PathPlanner::PathPlanner(GPS min, GPS max, std::vector<std::vector<double>> rawCostMap) :
  m_min (min),
  m_max (max)
{
    // Initialize the start and goal member variables
    
    //std::cout << "c min: "; m_min.print();
    //std::cout << "c max: "; m_max.print();

    // Initialize the raw data for the cost map, which is the 2d field of gradients. This will
    // be converted into an actual cost map of Location objects below. 
    std::vector<std::vector<double>> m_costMap;
    setCostMap(rawCostMap);

    std::cout << "Created PathPlanner instance\n";
}
PathPlanner::PathPlanner(GPS min, GPS max, GPS start, GPS goal) {
    m_min = min;
    m_max = max;
    m_start = start;
    m_gps_goal = goal;
}

PathPlanner::~PathPlanner() {
    //std::cout << "Destructing object\n";
    /*if (m_costMap != nullptr) {
        delete m_costMap;
        m_costMap = nullptr;
    }*/
}

double PathPlanner::get_gradientScore(Location current) {
    // Get the cost value from the current cell
    return current.gradientScore;
}

// Estimate remaining cost
double PathPlanner::get_heuristicScore(Location current) {
    // Get distance between current Location and m_goal, returning that distance
    return taxicab(current, m_goal);
}

/**
 * Return TRUE if heuristicScore is lower than previous hScore and was updated
 */
bool PathPlanner::calculate_totalScore(Location &current) {
    // Get partial scores
    double newGradient  = get_gradientScore(current); 
    double newHeuristic = get_heuristicScore(current);
    double newTotal = newGradient + newHeuristic;

    // If total score is lower, update
    if (newTotal < current.totalScore) {
        current.totalScore = newTotal;
        current.heuristicScore = newHeuristic;
        current.gradientScore = newGradient;
        return true;
    }
    return false;
}

/*double PathPlanner::get_totalScore(Location& current) {
    return current.totalScore;
}
*/


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
    startLocation.gradientScore = 0;
    startLocation.totalScore = calculate_totalScore(startLocation);
    openSet.push_back(startLocation);

    // Loop until our open set is empty
    while (openSet.size() > 0) {
        std::cout << "About to sleep\n";
        sleep(1);
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
                std::vector<int> pathPos {x,y};
                endPath.push_back(pathPos);

                // Move to the next location
                trackingPtr = trackingPtr->prev;
            }

            // Return the path 
            std::cout << "I'm a disapointment of a function!" << std::endl;
            return endPath;
        }
        std::cout << "was not the goal node\n";

        // Remove current from openSet, and add to closedSet
        std::cout << "Calling remove min\n";
        openSet = removeMin(openSet); // TODO Change openSet to be a min heap
        std::cout << "done with remove min\tSize: " << openSet.size() << "\n";
        for (int i=0; i<openSet.size(); i++) {
            std::cout << "  ";
            openSet[i].print();
        }

        closedSet.push_back(currentLocation);

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
                double temp_gradientScore = currentLocation.gradientScore + taxicab(currentLocation, neighbors[i]);
                std::cout << "temp_gradientScore: " << temp_gradientScore << "\n";
                
                // If our neighbor is not in the openSet, push to openSet
                if (!inSet(openSet, neighbors[i])) {
                    // Update scores
                    neighbors[i].gradientScore = temp_gradientScore;
                    neighbors[i].heuristicScore = get_heuristicScore(neighbors[i]);
                    calculate_totalScore(neighbors[i]);
                    // Push neighbor to openSet
                    openSet.push_back(neighbors[i]);
                    
                    std::cout << "Not in OpenSet\n";
                }
                // If, our neighbor is in the openSet, check if it is a better gradient score
                // if the gradient score is better, remove old Node and push new neighbor
                else {
                    // Linear probe to find previous occurance of neighbor
                    for (int j=0; j<openSet.size(); j++) {
                        // We found it!
                        if (openSet[j] == neighbors[i] && openSet[j] < neighbors[j]) {
                            openSet.erase(openSet.begin() + j);
                            // Insert new neighbor into openSet

                            // Update scores
                            neighbors[i].gradientScore = temp_gradientScore;
                            neighbors[i].heuristicScore = get_heuristicScore(neighbors[i]);
                            calculate_totalScore(neighbors[i]);
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

void PathPlanner::setCostMap(std::vector<std::vector<double>> rawCostMap) {
    // Loop through raw cost map, and create final cost map using getBoardIndex
    double delta_lat = (m_max.lat-m_min.lat) / rawCostMap[0].size();
    double delta_lon = (m_max.lon-m_min.lon) / rawCostMap.size();

    std::cout << "Raw cost map[0] size: " << rawCostMap[0].size() << std::endl;
    std::cout << "Raw cost map    size: " << rawCostMap.size() << std::endl;
    std::cout << "max.lat-min.lat: " << (m_max.lat-m_min.lat) << "\n";
    std::cout << "max.lon-min.lon: " << (m_max.lon-m_min.lon) << "\n";
    std::cout << "Delta lat: " << delta_lat << "\n";
    std::cout << "Delta lon: " << delta_lon << "\n";

    for (int i=0; i<rawCostMap.size(); i++) {
        std::vector<Location> locationLine = std::vector<Location>();
        for (int j=0; j<rawCostMap[i].size(); j++) {
            locationLine.push_back(Location(rawCostMap[i][j], delta_lat*i, delta_lon*j));
        }
        m_costMap.push_back(locationLine);
    }
    m_rawCostMap = rawCostMap;
}

Location& PathPlanner::getLocation(GPS point) {
    Location temp = Location(point);

    int x = -1,
        y = -1;
    std::cout<<"AAA\n";
    getBoardIndex(&temp, &x, &y);

    return m_costMap[x][y];
}
void PathPlanner::getBoardIndex(const Location* loc, int* p_x, int* p_y) {
    // Assign loc to temp variable so compiler doesn't get mad
    GPS temp = loc->m_gps;
    
    // Get proportion of GPS latitude between min and max latitude
    double lat_proportion = ((double)(temp.lat - m_min.lat))/(m_max.lat - m_min.lat);
    // Get proportion of GPS longitude between min and max longitude
    double lon_proportion = ((double)(temp.lon - m_min.lon))/(m_max.lon - m_min.lon);

    // Set x from latitute proportion
    *p_x = (int)(lat_proportion * (m_costMap[0].size()-1) );
    // Set y from longitude proportion
    *p_y = (int)(lon_proportion * (m_costMap.size()-1) );

    /*std::cout << "temp-m_min: " << (temp-m_min) << "\n";
    std::cout << "m_max-m_min: " << (m_max-m_min) << "\n";
    std::cout << "Lat p: " << lat_proportion << "\n";
    std::cout << "Lon p: " << lon_proportion << "\n";
    std::cout << "Temp: "; temp.print();
    std::cout << "Max: "; m_max.print();
    std::cout << "Min: "; m_min.print();
    std::cout << "\n";
    */
}

void PathPlanner::updatePartOfCostMap(std::vector<std::vector<double>> &costMap, int x_offset, int y_offset) {
    for (int i=0; i<costMap.size(); i++) {
        for (int j=0; j<costMap[i].size(); j++) {
            m_costMap[i+x_offset][j+y_offset] = costMap[i][j];
        }
    }
}
Location PathPlanner::getMin(std::vector<Location> &set) {
    // Find the Location with the lowest total score from the open set
	
	Location min = set[0];
	for (int i = 0; i < set.size(); i++)
	{
		if (set[i].totalScore < min.totalScore)
		{
			min = set[i];
		}
	}

    std::cout << "getMin: ";
    min.print();
	
    return min;
}
std::vector<Location> PathPlanner::removeMin(std::vector<Location> set) {
    // Find and remove the Location with minimum f score from the open set
	
	int minIndex = 0;
	for (int i = 0; i < set.size(); i++)
	{
        // Standard linear scan
		if (set[i].totalScore < set[minIndex].totalScore)
		{
			minIndex = i; 
		}
	}
	
    std::cout << "removeMin index: " << minIndex << "\tsize before erase: " << set.size() << "\n";
	set.erase(set.begin() + minIndex);
    std::cout << "size after erase: " << set.size() << "\n";
    return set;
}

bool PathPlanner::inSet(std::vector<Location> &set, Location& entry) {
    for (int i=0; i<set.size(); i++) {
        if (set[i] == entry) {
            std::cout << "####\n";
            entry.print();
            std::cout << "FOUND ENTRY\n";
            std::cout << "####\n";
            return true;
        }
    }
    return false;
    //return std::find(set.begin(), set.end(), entry) != set.end();
}

std::vector<Location> PathPlanner::getNeighbors(Location node) {
    // Return the north, east, south and west neighbors of the current Location node
	
    // Completed March 17, not yet tested 
	std::vector<Location> neighbors;

    // Assume a 2D array and access the four neighbors. Assume that the cost map will contain 
    // location objects, and that these location objects are located in the array according
    // to there x and y member variables. 
    int x = -1;
    int y = -1;
    std::cout<<"BBB\n";
    getBoardIndex(&node, &x, &y);
    std::cout << "getNeighbors of X: " << x << "\tY: " << y << "\n";
    if (y < m_costMap[0].size()) {
        neighbors.push_back(m_costMap[x  ][y+1]);
    }
    if (x < m_costMap.size()) {
        neighbors.push_back(m_costMap[x+1][y  ]);
    }
    if (y > 0) {
        neighbors.push_back(m_costMap[x  ][y-1]);
    }
    if (x > 0) {
        neighbors.push_back(m_costMap[x-1][y  ]);
    }

    return neighbors;
}

unsigned PathPlanner::taxicab(Location start, Location end) {
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
