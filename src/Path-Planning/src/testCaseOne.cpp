#include "path_planner.hpp"
#include <iostream>
#include <vector>

using namespace std;

void testCaseThree() {
    // Testing Location assignment
    Location a ( GPS(1,1), 3 );
    Location b ( GPS(2,2), 4 );
    a.print();
    b.print();
    a = b;
    a.print();
    b.print();
}

void testCaseFour() {
    // Testing inset
    std::vector<Location> vec;
    Location a ( GPS(1,1), 3 );
    Location b ( GPS(2,2), 4 );
    Location c ( GPS(5,6), 7 );
    vec.push_back(a);
    vec.push_back(b);

    if (inSet(vec,a))
        std::cout << "A in set\n";
    if (inSet(vec,b))
        std::cout << "B in set\n";
    if (inSet(vec,c))
        std::cout << "C in set\n";
}

void testCaseTwo()
{
    // Find the data
    vector<vector<double>> gradientMap = {
        {9, 8, 9, 7, 8, 9, 1, 1},
        {9, 8, 9, 7, 9, 8, 1, 7},
        {7, 7, 9, 9, 7, 1, 1, 7},
        {9, 8, 7, 7, 9, 1, 9, 8},
        {9, 7, 1, 1, 1, 1, 9, 7},
        {8, 9, 1, 8, 7, 8, 9, 7},
        {7, 1, 1, 9, 8, 9, 7, 9},
        {1, 1, 9, 7, 7, 9, 8, 8}
    };

    GPS startGPS(0.0, 0.0);
    GPS endGPS (7.19457e-5, 7.19457e-5);
    GPS currentGPS = startGPS;

    PathPlanner testPath(startGPS, endGPS, gradientMap);

    // Testing vector functions
    std::vector<Location> locs;
    locs.push_back( Location(GPS(2,1) , 3) );
    locs.push_back( Location(GPS(1,1) , 5) );
    locs.push_back( Location(GPS(3,1) , 2) );
    locs.push_back( Location(GPS(-1,4), 4) );
    locs.push_back( Location(GPS(5,1) , 1) );

    // Setting totalScores to be the cost
    for (int i=0; i<locs.size(); i++) {
        locs[i].setTotalScore(locs[i].getCost());
    }

    std::cout << "Printing vector\n";
    for (int i=0; i<locs.size(); i++) {
        locs[i].print();
    }

    for (int j=0; j<3; j++) {
    std::cout << "\n\nDeleting min:\n";
    removeMin(locs);

    std::cout << "Printing vector\n";
    for (int i=0; i<locs.size(); i++) {
        locs[i].print();
    }
    }
}

void testCaseOne()
{
    // Find the data
    vector<vector<double>> gradientMap = {
        {1, 1, 9, 7, 7, 9, 8, 8},
        {7, 1, 1, 9, 8, 9, 7, 9},
        {8, 9, 1, 8, 7, 8, 9, 7},
        {9, 7, 1, 1, 1, 1, 9, 7},
        {9, 8, 7, 7, 9, 1, 9, 8},
        {7, 7, 9, 9, 7, 1, 1, 7},
        {9, 8, 9, 7, 9, 8, 1, 7},
        {9, 8, 9, 7, 8, 9, 1, 1}
    };

    GPS startGPS(0.0, 0.0);
    GPS endGPS (7.19457e-5, 7.19457e-5);
    GPS currentGPS = startGPS;


    PathPlanner testPath(startGPS, endGPS, gradientMap);

    std::cout << "Test Path Parameters...\n";
    std::cout << "start: "; startGPS.print();
    std::cout << "end: "  ;   endGPS.print();
    std::vector<std::vector<int>> endPath = testPath.planPath(startGPS, endGPS);
    cout << "endPath: " << endPath.size() << endl;

    for (int i = 0; i < endPath.size(); i++)
    {
        for (int j = 0; j < endPath[i].size(); j++)
        {
            cout << endPath[i][j] << "  ";
        }
        cout << endl;
    }
}

int main() {
#if 1
    testCaseOne();
#else
    //testCaseTwo();
    //testCaseThree();
    testCaseFour();
#endif

    return 0;
}
