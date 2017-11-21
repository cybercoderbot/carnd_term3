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
    // define wp spline trajectory
    tk::spline wp_spline_x_;
    tk::spline wp_spline_y_;
    tk::spline wp_spline_dx_;
    tk::spline wp_spline_dy_;

public:
    Map(){};
    Map(std::string map_file_);
    ~Map() {};

    std::vector<double> getXY(double s, double d);

};


#endif // MAP_H
