#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>  // atof


using std::cout;
using std::cin;
using std::string;
using std::vector;


int main(void)
{
  vector<vector<double> > gradientMap;

  //const string fileName = "testCostMap.txt";
  std::ifstream inFile;
  inFile.open("testCostMap.txt");

  cout << std::atof("100") << "\n";

  if (inFile.is_open())
  {
    vector<double> data;

    while (!inFile.eof())
    {
      string temp;

      inFile >> temp;
      cout << temp << "\n";
      double thisData = std::atof(temp);
      data.push_back(thisData);

      //getline(inFile, row);

    }

    for (int i = 0; i < data.size(); i++)
    {
      cout << data[i] << " ";
    }
  }



  return 0;
}
