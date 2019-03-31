#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_

#include <vector>
#include <limits>
#include <math.h>
#include <cmath>
#include <cstdlib>
#include <string>

#include <iostream>

const double pi = acos(-1);

/**
 * Struct to store GPS coordinate
 */
class GPS {
    public:
        double lat; /// Stores latitude in radians
        double lon; /// Stores longitude in radians

        GPS() : GPS(-1.0,-1.0) {}

        GPS(double latitude, double longitude, bool isDegrees=true) {
            //printf("GPS: lat: %.14f\t lon: %.14f\n", latitude, longitude);
            // Input should always be in degrees, convert to radians for processing
            if (isDegrees) {
                lat = degToRad(latitude);
                lon = degToRad(longitude);
            } else {
                lat = latitude;
                lon = longitude;
            }
            //printf("GPS: lat: %.14f\t lon: %.14f\n", lat, lon);
        }

        // Copy constructor
        GPS(const GPS &rhs) {
            this->lat = rhs.lat;
            this->lon = rhs.lon;
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
            double temp = fabs(this->lat - rhs.lat) + fabs(this->lon - rhs.lon);
            return temp < 1e-16;
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
        double calcXDiff(const GPS &newGPS)
        {
            GPS temp = newGPS;
            temp.lon = this->lon;
            return (*this - temp);
        }

        double calcYDiff(const GPS &newGPS)
        {
            GPS temp = newGPS;
            temp.lat = this->lat;
            return (*this - temp);
        }
};

/**
 * Struct to keep track of positions
 */
class Location : public GPS {
    public:
        // Constructors
        Location(GPS point, double cost) :
            GPS(point)
        {
            totalScore     = std::numeric_limits<double>::max();
            gradientScore  = std::numeric_limits<double>::max();
            heuristicScore = std::numeric_limits<double>::max();
            m_cost = cost;
            prev = nullptr;
        }
        // May copy previous pointer?
        /*Location(const Location &rhs) {

          }*/
        bool operator>(const Location &rhs) const {
            return this->totalScore > rhs.totalScore;
        }
        bool operator<(const Location &rhs) const {
            return this->totalScore < rhs.totalScore;
        }
        bool operator==(const GPS &rhs) const {
            return ((GPS)*this) == rhs;
        }
        bool operator!=(const Location &rhs) const {
            return !(*this == rhs);
        }
        Location operator=(const Location &rhs) {
            // Does not copy prev pointer
            /*Location temp = Location();
              temp.m_gps    = rhs.m_gps;
              temp.totalScore    = rhs.totalScore;
              temp.gradientScore = rhs.gradientScore;
              temp.heuristicScore = rhs.heuristicScore;
              temp.m_cost = rhs.m_cost;*/
            //(GPS)*this = (GPS)rhs;
            lat = rhs.lat;
            lon = rhs.lon;
            totalScore = rhs.totalScore;
            gradientScore = rhs.gradientScore;
            heuristicScore = rhs.heuristicScore;
            m_cost = rhs.m_cost;
            return rhs;
        }
        void print() {
            printf("Location printing...\n  ");
            //((GPS)*this).print();
            printf("Lat: %.14f, Lon: %.14f\n",lat,lon);
            //printf("total: %10f\tgradient: %10f\theuristic: %10f\tcost: %10f\n", totalScore, gradientScore, heuristicScore, m_cost);
            printf("  cost: %f\n",m_cost);
        }

        // Accessors
        double getTotalScore() { return totalScore; }
        double getGradientScore() { return gradientScore; }
        double getHeuristicScore() { return heuristicScore; }
        double getCost() { return m_cost; }
        Location* getPrev() { return prev; }

        // Modifiers
        void setTotalScore() {
            totalScore = getGradientScore() + getHeuristicScore();
        }
        void setTotalScore(double testing) {
            totalScore = testing;
        }
        void setGradientScore(double score) {
            gradientScore = score;
        }
        void setHeuristicScore(double score) {
            heuristicScore = score;
        }

        // Cleanup
        void cleanup() {
            totalScore = 0;
            gradientScore = 0;
            heuristicScore = 0;
            prev = nullptr;
        }
    private:
        double totalScore;
        double gradientScore;
        double heuristicScore;

        double m_cost;

        Location* prev;
};


/**
 * Class to return the trajectory for the rover along a path given a destination GPS coordinate
 */
class PathPlanner {
    public:
        // Constructors
        //PathPlanner();
        PathPlanner(GPS start, GPS goal, std::vector<std::vector<double>> rawCostMap);
        //PathPlanner(GPS min, GPS max, GPS start, GPS goal);

        // Destructors
        ~PathPlanner();

        // Heuristics
        // Helper functions to calculate scores
        bool calculate_totalScore(Location &current);
        bool calculate_gradientScore(Location &target, Location &current);
        bool calculate_heuristicScore(Location &current);

        // Path plan
        // Return trajectory of found path
        std::vector<std::vector<int>> planPath(GPS current, GPS goal);

        // Update cost map
        void setCostMap(std::vector<std::vector<double>> costMap);
        //void updatePartOfCostMap(double* costMap, int x_min, int x_max, int y_min, int y_max);
        //void updatePartOfCostMap(std::vector<std::vector<double>> &costMap, int x_offset, int y_offset);

        // Conversion between GPS and board index
        Location& getLocation(GPS point);
        void getBoardIndex(const GPS* loc, int* p_x, int* p_y);

        // Accessors
        double get_gradientScore(Location current);
        double get_heuristicScore(Location current);
        double get_totalScore(Location current);

        // Modifiers

    private:
        // Store these in radians
        // Keep track of extreme points on graph
        GPS m_min;
        GPS m_max;

        // Position data
        GPS m_start;             // GPS coordinate of original start, for distance calculation
        GPS m_goal;              // GPS coordinate of original goal, for distance calculation

        // Current goal data
        GPS m_currentGoal;

        // Cost map
        std::vector<std::vector<Location>> m_costMap;
        //std::vector<std::vector<double>> m_rawCostMap;

        // Path solution
        std::vector<Location> m_steps;

        // Score consts
        double gWeight;
        double hWeight;

        // Helper functions
        std::vector<Location> getNeighbors(Location &node);
        double taxicab(GPS start, GPS end);
};

Location getMin(std::vector<Location> &vec);
//std::vector<Location> removeMin(std::vector<Location> set);
void removeMin(std::vector<Location> &vec);
bool inSet(std::vector<Location> &set, Location &entry);

#endif
