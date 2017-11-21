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
  void updateTrajectory(Map& map, std::vector<std::vector<double>>& trajectory);
  void createTrajectory(Map& map, Road& road, Vehicle& car, std::vector<std::vector<double>>& trajectory);

  void setState(LANE current_lane, LANE target_lane);

  /*  Actions */
  void applyAction(Vehicle& car, LANE current_lane, LANE target_lane);
  void startCar(Vehicle& car);
  void stayInLane(Vehicle& car);
  void reduceSpeed(Vehicle& car);
  void changeLane(Vehicle& car, LANE target_lane);

};

#endif /* PLANNER_H */
