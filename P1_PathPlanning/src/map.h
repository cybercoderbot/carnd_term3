#ifndef MAP_H
#define MAP_H

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>

#include "spline.h"

using namespace std;
using namespace tk; // spline

class Map {

  protected:
    // define wp spline trajectory
    spline wp_spline_x_;
    spline wp_spline_y_;
    spline wp_spline_dx_;
    spline wp_spline_dy_;

  public:
    Map(){};
    Map(string map_file_);
    ~Map() {};

    std::vector<double> getXY(double s, double d);

};

#endif // ROAD_H
