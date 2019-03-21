#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include <vector>
#include <limits>
#include <math.h>
#include <cstdlib>
#include <string>

#include <iostream>

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
        printf("GPS: lat: %.14f\t lon: %.14f\n", latitude, longitude);
        // Input should always be in degrees, convert to radians for processing
        lat = degToRad(latitude);
        lon = degToRad(longitude);
        printf("GPS: lat: %.14f\t lon: %.14f\n", lat, lon);
    }

    void print() {
        printf("Lat: %.14f, Lon: %.14f\n",lat,lon);
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

    bool operator==(const GPS &rhs) const {
        double temp = abs(this->lat - rhs.lat) + abs(this->lon - rhs.lon);
        return temp < 1e-20;
    }

    GPS operator=(const GPS &rhs) 
    {
        lat = rhs.lat;
        lon = rhs.lon;
        return rhs;
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
    GPS m_gps;
    double totalScore;
    double gradientScore;
    double heuristicScore;
    double m_cost;
    Location* prev;
    Location():
        Location(0, 0, 0)
    {
    }
    Location(GPS gps_point):
        Location(0, gps_point)
    {
        /*totalScore     = std::numeric_limits<double>::max();
        gradientScore  = std::numeric_limits<double>::max();
        heuristicScore = std::numeric_limits<double>::max();
        m_cost = 0;
        prev = nullptr;
        std::cout << "Pre assign Location: "; m_gps.print();
        //m_gps = GPS(gps_point.lat, gps_point.lon);
        m_gps = gps_point;
        std::cout << "Post assign Location: "; m_gps.print();
        std::cout << "Passed Location: "; gps_point.print();
        */
    }
    Location(double cost):
        Location(cost, 0, 0)
    {
        /*
        totalScore     = std::numeric_limits<double>::max();
        gradientScore  = std::numeric_limits<double>::max();
        heuristicScore = std::numeric_limits<double>::max();
        m_cost = cost;
        prev = nullptr;
        */
    }
    Location(double cost, double lat, double lon):
        Location(cost, GPS(lat, lon))
    {
    }
    Location(double cost, GPS point)
    {
        totalScore     = std::numeric_limits<double>::max();
        gradientScore  = std::numeric_limits<double>::max();
        heuristicScore = std::numeric_limits<double>::max();
        m_cost = 0;
        prev = nullptr;
        std::cout << "Pre assign Location: "; m_gps.print();
        //m_gps = GPS(lat, lon);
        m_gps = point;
        std::cout << "Post assign Location: "; m_gps.print();
    }
    bool operator>(const Location &rhs) const {
        return this->totalScore > rhs.totalScore;
    }
    bool operator<(const Location &rhs) const {
        return this->totalScore < rhs.totalScore;
    }
    bool operator==(const Location &rhs) const {
        return this->m_gps == rhs.m_gps;
    }
    Location operator=(const Location &rhs) {
        // Does not copy prev pointer
        /*Location temp = Location();
        temp.m_gps    = rhs.m_gps;
        temp.totalScore    = rhs.totalScore;
        temp.gradientScore = rhs.gradientScore;
        temp.heuristicScore = rhs.heuristicScore;
        temp.m_cost = rhs.m_cost;*/
        m_gps = rhs.m_gps;
        totalScore = rhs.totalScore;
        gradientScore = rhs.gradientScore;
        heuristicScore = rhs.heuristicScore;
        m_cost = rhs.m_cost;
        return rhs;
    }
    void print() {
        m_gps.print();
        //printf("total: %10f\tgradient: %10f\theuristic: %10f\tcost: %10f\n", totalScore, gradientScore, heuristicScore, m_cost);
        printf("cost: %f\n",m_cost);
    }
};


/**
 * Class to return the trajectory for the rover along a path given a destination GPS coordinate
 */
class PathPlanner {
    public:
        // Constructors
        //PathPlanner();
        PathPlanner(GPS start, GPS goal, std::vector<std::vector<double>> rawCostMap);
        PathPlanner(GPS min, GPS max, GPS start, GPS goal);

        // Destructors
        ~PathPlanner();

        // Heuristics
        // Helper functions to calculate scores
        bool calculate_totalScore(Location &current);
        //double get_totalScore(Location& current);

        // Path plan
        // Return trajectory of found path
        std::vector<std::vector<int>> planPath(GPS current, GPS goal);

        // Update cost map
        void setCostMap(std::vector<std::vector<double>> costMap);
        //void updatePartOfCostMap(double* costMap, int x_min, int x_max, int y_min, int y_max);
        void updatePartOfCostMap(std::vector<std::vector<double>> &costMap, int x_offset, int y_offset);

        // Conversion between GPS and board index
        Location& getLocation(GPS point);
        void getBoardIndex(const Location* loc, int* p_x, int* p_y);

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
        std::vector<std::vector<Location>> m_costMap;
        std::vector<std::vector<double>> m_rawCostMap;

        // Path solution
        std::vector<Location> m_steps;

        // Score consts
        double gWeight;
        double hWeight;

        // Helper functions
        Location getMin(std::vector<Location> &set);
        std::vector<Location> removeMin(std::vector<Location> set);
        bool inSet(std::vector<Location> &set, Location &entry);
        std::vector<Location> getNeighbors(Location node);
        unsigned taxicab(Location start, Location end);

        // Scorers
        double get_gradientScore(Location current);
        double get_heuristicScore(Location current);
};

#endif
