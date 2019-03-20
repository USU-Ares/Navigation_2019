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

/*
** Notes
** 1: A path planner instance will be created with the start and ending GPS coordinates,
** but a raw cost map will need to be sent in many times. Should the raw cost map be changed 
** so that it no longer is a parameter for the sonstructor? Is that what the second constructor
** is for?
**
** 2: Still need to integrate this with ROS. rawCostMap will be found with a ROS subscriber node. 
*/

PathPlanner::PathPlanner(GPS start, GPS goal, std::vector<std::vector<double>> rawCostMap) {
    // Initialize the start and goal member variables
    m_start = start;
    m_gps_goal = goal;

    // Initialize the raw data for the cost map, which is the 2d field of gradients. This will
    // be converted into an actual cost map of Location objects below. 
    m_rawCostMap = rawCostMap;

    std::cout << "Created PathPlanner instance\n";
}
PathPlanner::PathPlanner(GPS min, GPS max, GPS start, GPS goal) {
    m_min = min;
    m_max = max;
    m_start = start;
    m_gps_goal = goal;
}

PathPlanner::~PathPlanner() {
    std::cout << "Destructing object\n";
    /*if (m_costMap != nullptr) {
        delete m_costMap;
        m_costMap = nullptr;
    }*/
}

double PathPlanner::get_fScore(Location current) {
    // Get the cost value from the current cell
    return current.fScore;
}

// Estimate remaining cost
double PathPlanner::get_gScore(Location current) {
    // Get distance between current Location and m_goal, returning that distance
    return taxicab(current, m_goal);
}

/**
 * Return TRUE if hScore is lower than previous hScore and was updated
 */
bool PathPlanner::get_hScore(Location &current) {
    // Get partial scores
    double newFScore = get_fScore(current); 
    double newGScore = get_gScore(current);

    // If scores are lower than current score, replace them
    if (newFScore < current.fScore) {
        current.fScore = newFScore;
    }
    if (newGScore < current.gScore) {
        current.gScore = newGScore;
    }
    // Set hScore to be the sum of F and G if better, and return whether or not it was reassigned
    if (newFScore + newGScore > current.hScore) {
        current.hScore = current.fScore + current.gScore;
        return true;
    }
    return false;
}

double PathPlanner::get_hScore(Location& current, Location& goal) {
    return get_fScore(current) + get_gScore(current);
}



std::vector<std::vector<int>> PathPlanner::planPath(GPS currentGPS) {
    // Get current coordinate on the grid
    Location currentLocation = getLocation(currentGPS);

    // Sets of nodes that have been visited or to be checked
    std::vector<Location> closedSet;
    std::vector<Location> openSet;
    //std::priority_queue <Location, std::vector<Location>, std::greater<Location>> openSet;

    // Inital condition
    currentLocation.gScore = 0;
    currentLocation.fScore = get_fScore(currentLocation);
    openSet.push_back(currentLocation);

    // Loop until our open set is empty
    Location currentNode;
    Location goalNode = getLocation(m_gps_goal);

    while (openSet.size() > 0) {
        // Get lowest cost item from openSet
        currentNode = getMin(openSet);

        // Check if we have reached the goal
        if (currentNode == goalNode) {
            // Complete March 9, not tested
            // Reconstruct path

            std::vector<std::vector<int>> endPath;

            Location* trackingPtr = &currentNode;
            while (trackingPtr != nullptr)
            {
                // Get current location position
                int x, y;
                getBoardIndex(trackingPtr, &x, &y);

                // Add location to path vector
                std::vector<int> pathPos {x,y};
                endPath.push_back(pathPos);

                // Move to the next location
                trackingPtr = trackingPtr->prev;
            }

            // Return the path 
            return endPath;
        }

        // Remove current from openSet, and add to closedSet
        removeMin(openSet);
        closedSet.push_back(currentNode);

        // Check each neighbor of currentNode
        std::vector<Location> neighbors = getNeighbors(currentNode);

        for (int i=0; i<neighbors.size(); i++) {
            // Check if neighbor is not in the closed set, and analyze the nodes that are not.
            if (!inSet(closedSet, neighbors[i]))
            {
                // Distance from start to neighbor
                double temp_gScore = currentNode.gScore + taxicab(currentNode, neighbors[i]);

                // Check if neighbor not in openSet
                if (!inSet(openSet, neighbors[i])) 
                {
                    // Store current node as the previous location
                    neighbors[i].prev = &currentNode;

                    // Save the f, g, and h scores
                    neighbors[i].gScore = temp_gScore;
                    neighbors[i].hScore = get_hScore(currentNode, goalNode);
                    neighbors[i].fScore = temp_gScore + neighbors[i].hScore;

                    // If better than previous, add to set
                    openSet.push_back(neighbors[i]);
                }
            }
        }
    }
}

void PathPlanner::setCostMap(std::vector<std::vector<double>> rawCostMap) {
    // Loop through raw cost map, and create final cost map using getBoardIndex
    for (int i=0; i<rawCostMap.size(); i++) {
        std::vector<Location> locationLine = std::vector<Location>();
        for (int j=0; j<rawCostMap[i].size(); j++) {
            locationLine.push_back(Location(rawCostMap[i][j]));
        }
        m_costMap.push_back(locationLine);
    }
    m_rawCostMap = rawCostMap;
}

Location& PathPlanner::getLocation(GPS point) {
    Location temp = Location();
    temp.gps = point;

    int x = -1,
        y = -1;
    getBoardIndex(&temp, &x, &y);

    return m_costMap[x][y];
}
void PathPlanner::getBoardIndex(const Location* loc, int* p_x, int* p_y) {
    // Assign loc to temp variable so compiler doesn't get mad
    GPS temp = loc->gps;
    // Get proportion of GPS latitude between min and max latitude
    double lat_proportion = (double)(temp - m_min)/(m_max - m_min);
    // Get proportion of GPS longitude between min and max longitude
    double lon_proportion = (double)(temp - m_min)/(m_max - m_min);

    // Set x from latitute proportion
    *p_x = (int)(lat_proportion * m_costMap[0].size());
    // Set y from longitude proportion
    *p_y = (int)(lat_proportion * m_costMap[0].size());
}

void PathPlanner::updatePartOfCostMap(std::vector<std::vector<double>> &costMap, int x_offset, int y_offset) {
    for (int i=0; i<costMap.size(); i++) {
        for (int j=0; j<costMap[i].size(); j++) {
            m_costMap[i+x_offset][j+y_offset] = costMap[i][j];
        }
    }
}
Location PathPlanner::getMin(std::vector<Location> &set) {
    // Find the Location with the lowest f score from the open set
	
	Location min = set[0];
	for (int i = 0; i < set.size(); i++)
	{
		if (set[i].fScore < min.fScore)
		{
			min = set[i];
		}
	}
	
    return min;
}
void PathPlanner::removeMin(std::vector<Location> &set) {
    // Find and remove the Location with minimum f score from the open set
	
	int minIndex = 0;
	for (int i = 0; i < set.size(); i++)
	{
        // Standard linear scan
		if (set[i].fScore < set[minIndex].fScore)
		{
			minIndex = i; 
		}
	}
	
	set.erase(set.begin() + minIndex);
	return;

}

bool PathPlanner::inSet(std::vector<Location> &set, Location& entry) {
    return std::find(set.begin(), set.end(), entry) != set.end();
}

std::vector<Location> PathPlanner::getNeighbors(const Location &node) {
    // Return the north, east, south and west neighbors of the current Location node
	
    // Completed March 17, not yet tested 
	std::vector<Location> neighbors;

    // Assume a 2D array and access the four neighbors. Assume that the cost map will contain 
    // location objects, and that these location objects are located in the array according
    // to there x and y member variables. 
    int x = -1;
    int y = -1;
    getBoardIndex(&node, &x, &y);
    neighbors.push_back(m_costMap[x  ][y+1]);
    neighbors.push_back(m_costMap[x+1][y  ]);
    neighbors.push_back(m_costMap[x  ][y-1]);
    neighbors.push_back(m_costMap[x-1][y  ]);

    return neighbors;
}

unsigned PathPlanner::taxicab(Location start, Location end) {
    int x_1, y_1;
    int x_2, y_2;
    getBoardIndex(&start, &x_1, &y_1);
    getBoardIndex(&end  , &x_2, &y_2);
    // Get taxicab distance between two locations
    unsigned deltaX = abs(x_1 - x_2);
    unsigned deltaY = abs(y_1 - y_2);

    // Return taxicab distance
    return deltaX + deltaY;
}
