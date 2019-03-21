#include "path_planner.hpp"
#include <iostream>
#include <vector>


using namespace std;

int main(void)
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
  GPS endGPS(1.0, 1.0);
  std::cout << "start: "; startGPS.print();
  std::cout << "end: "; endGPS.print();
  GPS currentGPS = startGPS;
  
  cout << "hello" << endl;
  PathPlanner testPath(startGPS, endGPS, gradientMap);
  cout << "hello" << endl;
  std::vector<std::vector<int>> endPath = testPath.planPath(currentGPS, endGPS);
  cout << "endPath: " << endPath.size() << endl;
  
  
  
  for (int i = 0; i < endPath.size(); i++)
  {
    for (int j = 0; j < endPath[i].size(); j++)
    {
      cout << endPath[i][j] << "  ";
    }
    cout << endl;
  }
  
  
  
  return 0;
}