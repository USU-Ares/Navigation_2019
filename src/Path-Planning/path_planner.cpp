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


PathPlanner::PathPlanner(GPS start, GPS goal, std::vector<float> rawCostMap) {
    // TODO
    // Initialize the start and goal member variables
    m_start = start;
    m_gps_goal = goal;

    // Initialize the raw data for the cost map, which is the 2d field of gradients. This will
    // be converted into an actual cost map of Location objects below. 
    m_rawCostMap = rawCostMap

    std::cout << "Created PathPlanner instance\n";
}
PathPlanner::PathPlanner(GPS min, GPS max, GPS start, GPS goal) {
    // TODO
    m_min = min;
    m_max = max;
    m_start = start;
    m_gps_goal = goal;
}

PathPlanner::~PathPlanner() {
    std::cout << "Destructing object\n";
    if (m_costMap != nullptr) {
        delete m_costMap;
        m_costMap = nullptr;
    }
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



std::vector<int> PathPlanner::planPath(GPS currentGPS) {
    // Get current coordinate on the grid
    Location currentLocation = getBoardIndex(currentGPS);

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
    Location goalNode = getBoardIndex(m_gps_goal);

    while (openSet.size() > 0) {
        // Get lowest cost item from openSet
        currentNode = getMin(openSet);

        // Check if we have reached the goal
        if (currentNode.x == goalNode.x && currentNode.y == goalNode.y) {
            // TODO
            // Complete March 9, not tested
            // Reconstruct path

            std::vector<int> endPath;

            Location* trackingPtr = &currentNode;
            while (trackingPtr != nullptr)
            {
                // Get current location position
                std::vector<int> pathPos = {trackingPtr->x, trackingPtr->y};
                endPath.push_back(pathPos)

                // Move to the next location
                trackingPtr = trackingPtr->prev
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



            /* Old version, made more sense without the continue;

            if (inSet(closedSet, neighbors[i])) {
                continue;
            }

            // Distance from start to neighbor
            double temp_gScore = currentNode.gScore + taxicab(currentNode, neighbors[i]);

            // Check if neighbor not in openSet
            if (!inSet(openSet, neighbors[i])) {
                // Store current node as the previous location
                neighbors[i].prev = &currentNode;
                neighbors[i].gScore = temp_gScore;
                neighbors[i].hScore = get_hScore(currentNode, goalNode);
                neighbors[i].fScore = temp_gScore + neighbors[i].hScore;
                openSet.push_back(neighbors[i]);
            } */
            

        }
    }
}


void PathPlanner::initializeCostMap(unsigned int height, unsigned int width) {
    m_height = height;
    m_width  = width;

    m_costMap = new double[m_height*m_width]; // Create map

    // Zero the array
    for (unsigned int i=0; i<m_height*m_width; i++) {
        m_costMap[i] = 0;
    }
}

void PathPlanner::setCostMap(double* costMap) {
    for (int i=0; i<m_height*m_width; i++) {
        m_costMap[i] = costMap[i];
    }
}

Location PathPlanner::getBoardIndex(GPS point) {
    // TODO

    return Location();
}
void PathPlanner::getBoardIndex(Location* loc) {
    // TODO

}
void PathPlanner::getGPSPoint(Location* loc) {
    // TODO

}

void PathPlanner::updatePartOfCostMap(double* costMap, int x_min, int x_max, int y_min, int y_max) {
    for (int i=x_min; i<x_max; i++) {
        for (int j=y_min; j<y_max; j++) {
            m_costMap[i*m_width + j] = costMap[(i-x_min)*m_width + (j-y_min)];
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
	//Location min = set[minIndex];
	for (int i = 0; i < set.size(); i++)
	{
        // Standard linear scan
		if (set[i].fScore < min.fScore)
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
	
	// TODO The structure containing the locations must be implemented first.
    // Completed March 17, not yet tested 
	std::vector<Location> neighbors;

    // Assume a 2D array and access the four neighbors. Assume that the cost map will contain 
    // location objects, and that these location objects are located in the array according
    // to there x and y member variables. 
    neighbors.push_back(m_costMap[node.x][node.y+1])
    neighbors.push_back(m_costMap[node.x+1][node.y])
    neighbors.push_back(m_costMap[node.x][node.y-1])
    neighbors.push_back(m_costMap[node.x-1][node.y])

    return neighbors;
}

unsigned PathPlanner::taxicab(Location start, Location end) {
    // Get taxicab distance between two locations
    unsigned deltaX = abs(start.x - end.x);
    unsigned deltaY = abs(start.y - end.y);

    // Return taxicab distance
    return deltaX + deltaY;
}
