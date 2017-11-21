#ifndef ROAD_H
#define ROAD_H

#include <string>
#include <vector>
#include <cmath>
#include "vehicle.h"
#include "utils.h"

using namespace std;

class Road {

  protected:
    std::vector<Vehicle> left_lane_;
    std::vector<Vehicle> center_lane_;
    std::vector<Vehicle> right_lane_;

  public:
    Road() {};
    ~Road() {};

    void update(std::vector<Vehicle> l, std::vector<Vehicle> c, std::vector<Vehicle> r);
    std::vector<Vehicle> getLaneStatus(LANE lane);

    bool isSafeLane(Vehicle& car, LANE lane);
    bool isFreeLane(Vehicle& car, LANE lane);
    LANE laneChangeAvailable(Vehicle& car);
};

#endif // ROAD_H
