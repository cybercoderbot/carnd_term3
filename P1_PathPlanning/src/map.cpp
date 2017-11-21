#include "map.h"

Map::Map(std::string mfile)
  : map_file_ (mfile)
{ 
}

std::vector<double> Map::getXY(double s, double d){

  std::vector<double> map_wp_x;
  std::vector<double> map_wp_y;
  std::vector<double> map_wp_s;
  std::vector<double> map_wp_dx;
  std::vector<double> map_wp_dy;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  std::string line;
  while (std::getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_wp_x.push_back(x);
    map_wp_y.push_back(y);
    map_wp_s.push_back(s);
    map_wp_dx.push_back(d_x);
    map_wp_dy.push_back(d_y);
  }


  // define wp spline trajectory
  tk::spline wp_spline_x_;
  tk::spline wp_spline_y_;
  tk::spline wp_spline_dx_;
  tk::spline wp_spline_dy_;

  wp_spline_x_.set_points(map_wp_s, map_wp_x);
  wp_spline_y_.set_points(map_wp_s, map_wp_y);
  wp_spline_dx_.set_points(map_wp_s, map_wp_dx);
  wp_spline_dy_.set_points(map_wp_s, map_wp_dy);

  double wp_x, wp_y; 
  double wp_dx, wp_dy;
  double next_x, next_y;

  // spline interpolation
  wp_x  = wp_spline_x_(s);
  wp_y  = wp_spline_y_(s);
  wp_dx = wp_spline_dx_(s);
  wp_dy = wp_spline_dy_(s);

  next_x = wp_x + wp_dx * d;
  next_y = wp_y + wp_dy * d;

  return {next_x, next_y};
}
