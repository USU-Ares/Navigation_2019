#include "path_planner.hpp"

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

void PathPlanner::updatePartOfCostMap(double* costMap, int x_min, int x_max, int y_min, int y_max) {
    for (int i=x_min; i<x_max; i++) {
        for (int j=y_min; j<y_max; j++) {
            m_costMap[i*m_width + j] = costMap[(i-x_min)*m_width + (j-y_min)];
        }
    }
}
