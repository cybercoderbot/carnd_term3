#include "map.h"

Map::Map(std::string map_file_) {

  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  std::string line;
  while (getline(in_map_, line)) {
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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // set points
  wp_spline_x_.set_points(map_waypoints_s, map_waypoints_x);
  wp_spline_y_.set_points(map_waypoints_s, map_waypoints_y);
  wp_spline_dx_.set_points(map_waypoints_s, map_waypoints_dx);
  wp_spline_dy_.set_points(map_waypoints_s, map_waypoints_dy);

}

std::vector<double> Map::getXY(double s, double d){
  double wp_x, wp_y, wp_dx, wp_dy, next_x, next_y;

  // spline interpolation
  wp_x  = wp_spline_x_(s);
  wp_y  = wp_spline_y_(s);
  wp_dx = wp_spline_dx_(s);
  wp_dy = wp_spline_dy_(s);

  next_x = wp_x + wp_dx * d;
  next_y = wp_y + wp_dy * d;

  return {next_x, next_y};
}
