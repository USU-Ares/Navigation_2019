#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include <vector>

/**
 * Struct to store GPS coordinate
 */
struct GPS {
    double lat; /// Stores latitude in degrees
    double lon; /// Stores longitude in degrees
    GPS() {
        lat = 0.0;
        lon = 0.0;
    }
};

/**
 * Enum to keep track of direction to go in map
 */
enum Direction {
    UP,
    DOWN,
    LEFT,
    RIGHT
};

/**
 * Class to return the heading for the rover along a path given a destination GPS coordinate
 */
class PathPlanner {
    public:
        // Constructors
        PathPlanner();
        PathPlanner(GPS min, GPS max, GPS start, GPS goal);
        
        // Destructors
        ~PathPlanner();

        // Heuristics
        // Helper functions to calculate scores
        double fScore(GPS current);
        double gScore(GPS current);
        double hScore(GPS current);

        // Path plan
        Direction planPath(GPS current);

        // Update cost map
        void initializeCostMap(unsigned int height, unsigned int width);
        void setCostMap(double* costMap);
        void updatePartOfCostMap(double* costMap, int x_min, int x_max, int y_min, int y_max);

        // Conversion between GPS and board index
        void getBoardIndex(GPS point, int& x, int& y);
        void getGPSPoint(GPS& point, int x, int y);

        // Accessors
        // Modifiers

    private:
        // Store these in radians
        // Keep track of extreme points on graph
        GPS m_min;
        GPS m_max;

        // Position data
        GPS m_start;        // GPS coordinate to begin path planning
        GPS m_current;      // Current GPS coordinate of rover
        GPS m_goal;         // GPS coordinate of goal
        GPS m_currentGoal;  // GPS coordinate of temporary goal for search pattern

        // Cost map
        double* m_costMap;
        unsigned int m_height;
        unsigned int m_width;

        // Path solution
        std::vector<Direction> steps;

        // Score consts
        double gWeight;
        double hWeight;
};

#endif
