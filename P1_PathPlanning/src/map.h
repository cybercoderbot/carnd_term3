#ifndef MAP_H
#define MAP_H

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>

#include "spline.h"

  
class Map {

private:
    std::string map_file_;

public:
    Map(){};
    Map(std::string mfile);
    ~Map() {};

    std::vector<double> getXY(double s, double d);

};

#endif // MAP_H
