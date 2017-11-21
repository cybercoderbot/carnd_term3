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

    void update_road(std::vector<Vehicle> l, std::vector<Vehicle> c, std::vector<Vehicle> r);
    std::vector<Vehicle> get_lane_status(LANE lane);

    bool safe_lane(Vehicle& car, LANE lane);
    bool free_lane(Vehicle& car, LANE lane);
    LANE lane_change_available(Vehicle& car);
};

#endif // ROAD_H
