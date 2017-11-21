#ifndef PLANNER_H
#define PLANNER_H

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include <vector>

#include "utils.h"
#include "map.h"
#include "road.h"

class Planner {

protected:

  int n_;
  STATE state_;
  std::vector<double> start_s_;
  std::vector<double> end_s_;
  std::vector<double> start_d_;
  std::vector<double> end_d_;
  bool new_points_;

public:
  Planner();
  ~Planner(){};

  std::vector<double> JMT(std::vector<double> start, std::vector <double> end, double T);
  void estimate_new_points(Map& map, std::vector<std::vector<double>>& trajectory);
  void create_trajectory(Map& map, Road& road, Vehicle& car, std::vector<std::vector<double>>& trajectory);

  void set_state(LANE current_lane, LANE target_lane);

  /*  Actions */
  void apply_action(Vehicle& car, LANE current_lane, LANE target_lane);
  void start_car(Vehicle& car);
  void stay_in_lane(Vehicle& car);
  void reduce_speed(Vehicle& car);
  void change_lane(Vehicle& car, LANE target_lane);

};

#endif /* PLANNER_H */
