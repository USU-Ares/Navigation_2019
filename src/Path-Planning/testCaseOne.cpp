#include "path_planner.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

using namespace std;

int main(void)
{
    // Find the data
    vector<vector<double>> gradientMap;

    ifstream inFile("gradienMap.txt");

    if (inFile.is_open())
    {
        string line;
        while (getline(inFile, line))
        {
            cout << line << endl;
        }

        inFile.close();
    }




    return 0;
}