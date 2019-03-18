#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include <vector>
#include <limits>
#include <math.h>


const double pi = acos(-1);

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
    
    GPS(double latitude, double longitude) {
        // Input should always be in degrees, convert to radians for processing
        lat = degToRad(latitude);
        lon = degToRad(longitude);
    }
    
    double degToRad(double angle)
    {
        // Convert degrees to radians
        return angle*(pi/180);   
    }
    
    double operator-(GPS rhs) {
        // Calculate distance to GPS point
        
        // Radius of Earth, in meters
        const double earthRadius = 6371e3;
        
        // Apply haversine function
        double haversine = hav(rhs.lat - lat) + cos(lat) * cos(rhs.lat) * hav(lon - rhs.lon);
        return 2 * earthRadius * asin(sqrt(haversine));
    }
    
    // Haversine function
    double hav(double angle) {
        return 0.5 - cos(angle)*0.5;
    }
    
    // Calculate the x and y difference
    double calcXDiff(GPS newGPS)
    {
      GPS temp = newGPS;
      temp.lon = this->lon;
      return (*this - temp);
    }
    
    double calcYDiff(GPS newGPS)
    {
      GPS temp = newGPS;
      temp.lat = this->lat;
      return (*this - temp);
    }
};

/**
 * Struct to keep track of positions
 */
struct Location {
    int x;
    int y;
    double fScore;
    double gScore;
    double hScore;
    Location* prev;
    Location() {
        fScore = std::numeric_limits<double>::max();
        gScore = std::numeric_limits<double>::max();
        hScore = std::numeric_limits<double>::max();
        x = -1;
        y = -1;
        prev = nullptr;
    }
    bool operator>(const Location &rhs) const {
        return this->fScore > rhs.fScore;
    }
    bool operator==(const Location &rhs) const {
        return this->x == rhs.x && this->y == rhs.y;
    }
};


/**
 * Class to return the trajectory for the rover along a path given a destination GPS coordinate
 */
class PathPlanner {
    public:
        // Constructors
        //PathPlanner();
        PathPlanner(GPS start, GPS goal, std::vector<float>);
        PathPlanner(GPS min, GPS max, GPS start, GPS goal);
        
        // Destructors
        ~PathPlanner();

        // Heuristics
        // Helper functions to calculate scores
        bool get_hScore(Location &current);
        double get_hScore(Location& current, Location& goal);

        // Path plan
        // Return trajectory of found path
        std::vector<int> planPath(GPS current);

        // Update cost map
        void initializeCostMap(unsigned int height, unsigned int width);
        void setCostMap(double* costMap);
        void updatePartOfCostMap(double* costMap, int x_min, int x_max, int y_min, int y_max);

        // Conversion between GPS and board index
        //void getBoardIndex(GPS point, int* x, int* y);
        Location getBoardIndex(GPS point);
        //void getGPSPoint(GPS& point, int x, int y);
        void getBoardIndex(Location* loc);
        void getGPSPoint(Location* loc);

        // Accessors
        // Modifiers

    private:
        // Store these in radians
        // Keep track of extreme points on graph
        GPS m_min;
        GPS m_max;

        // Position data
        GPS m_start;             // GPS coordinate to begin path planning
        GPS m_gps_current;       // Current GPS coordinate of rover
        GPS m_gps_goal;          // GPS coordinate of goal
        GPS m_currentGoal;       // GPS coordinate of temporary goal for search pattern
        
        // Array position data
        Location m_current;      // Current Location coordinate of rover
        Location m_goal;         // Location coordinate of goal

        // Cost map
        double* m_costMap;
        unsigned int m_height;
        unsigned int m_width;

        // Path solution
        std::vector<Direction> m_steps;

        // Score consts
        double gWeight;
        double hWeight;

        // Helper functions
        Location getMin(std::vector<Location> &set);
        void  removeMin(std::vector<Location> &set);
        bool inSet(std::vector<Location> &set, Location &entry);
        std::vector<Location> getNeighbors(const Location &node);
        unsigned taxicab(Location start, Location end);

        // Scorers
        double get_fScore(Location current);
        double get_gScore(Location current);
};

#endif
