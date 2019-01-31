#include "path_planner.hpp"
#include <bits/stdc++.h>
#include <algorithm>

PathPlanner::PathPlanner() {

}

PathPlanner::PathPlanner(GPS min, GPS max, GPS start, GPS goal) {
    m_min = min;
    m_max = max;
    m_start = start;
    m_goal  = goal;
}

PathPlanner::~PathPlanner() {
    if (m_costMap != nullptr) {
        delete m_costMap;
        m_costMap = nullptr;
    }
}

double PathPlanner::fScore(Location current) {
    return 0.0;
}
double PathPlanner::gScore(Location current) {
    return 0.0;
}
double PathPlanner::hScore(Location current) {
    return fScore(current) + gScore(current);
}
double PathPlanner::hScore(Location& current, Location& goal) {
    return fScore(current) + gScore(current);
}

double PathPlanner::planPath(GPS currentGPS) {
    // Get current coordinate on the grid
    Location currentLocation = getBoardIndex(currentGPS);

    // Sets of nodes that have been visited or to be checked
    std::vector<Location> closedSet;
    std::vector<Location> openSet;
    //std::priority_queue <Location, std::vector<Location>, std::greater<Location>> openSet;

    // Inital condition
    currentLocation.gScore = 0;
    currentLocation.fScore = fScore(currentLocation);
    openSet.push_back(currentLocation);

    // Loop until our open set is not empty
    Location currentNode;
    Location goalNode = getBoardIndex(m_goal);
    while (openSet.size() > 0) {
        // Get lowest cost item from openSet
        currentNode = getMin(openSet);

        // Check if we have reached the goal
        if (currentNode.x == goalNode.x && currentNode.y == goalNode.y) {
            // Construct path
        }

        // Remove current from openSet, and add to closedSet
        removeMin(openSet);
        closedSet.push_back(currentNode);

        // Check each neighbor of currentNode
        std::vector<Location> neighbors = getNeighbors(currentNode);

        for (int i=0; i<neighbors.size(); i++) {
            // Check if neighbor is in closed set
            if (inSet(closedSet, neighbors[i])) {
                continue;
            }

            // Distance from start to neighbor
            double temp_gScore = currentNode.gScore + dist_between(currentNode, neighbors[i]);

            // Check if neighbor not in openSet
            if (!inSet(openSet, neighbors[i])) {
                neighbors[i].prev = &currentNode;
                neighbors[i].gScore = temp_gScore;
                neighbors[i].hScore = hScore(currentNode, goalNode);
                neighbors[i].fScore = temp_gScore + neighbors[i].hScore;
                openSet.push_back(neighbors[i]);
            }
            // If better than previous, add to set

        }
    }

    return 0;
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

    return Location();
}
void PathPlanner::getBoardIndex(Location* loc) {

}
void PathPlanner::getGPSPoint(Location* loc) {

}

void PathPlanner::updatePartOfCostMap(double* costMap, int x_min, int x_max, int y_min, int y_max) {
    for (int i=x_min; i<x_max; i++) {
        for (int j=y_min; j<y_max; j++) {
            m_costMap[i*m_width + j] = costMap[(i-x_min)*m_width + (j-y_min)];
        }
    }
}
Location PathPlanner::getMin(std::vector<Location> &set) {
    return Location();
}
void PathPlanner::removeMin(std::vector<Location> &set) {

}

bool PathPlanner::inSet(std::vector<Location> &set, Location& entry) {
    return std::find(set.begin(), set.end(), entry) != set.end();
}
std::vector<Location> PathPlanner::getNeighbors(const Location &node) {

    return std::vector<Location>();
}
double PathPlanner::dist_between(Location start, Location end) {

    return 0;
}
