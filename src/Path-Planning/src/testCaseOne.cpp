#include "path_planner.hpp"
#include <iostream>
#include <vector>

using namespace std;

void testCaseOne()
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
    testCaseOne();

    return 0;
}
