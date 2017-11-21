#include "road.h"


void Road::update_road(std::vector<Vehicle> l, std::vector<Vehicle> c, std::vector<Vehicle> r){
  left_lane_ = l;
  center_lane_ = c;
  right_lane_ = r;
}

std::vector<Vehicle> Road::get_lane_status(LANE lane){
  std::vector<Vehicle> rlane;
  if (lane == LANE::LEFT) {
    rlane = left_lane_;
  } else if (lane == LANE::CENTER) {
    rlane = center_lane_;
  } else {
    rlane = right_lane_;
  }
  return rlane;
}

bool Road::safe_lane(Vehicle& car, LANE lane){
  std::vector<Vehicle> r_car_lane = get_lane_status(lane);
  bool safe = true;
  for (int i = 0; i < r_car_lane.size(); i++) {
    double distance = r_car_lane[i].get_s() - car.get_s();
    if(distance > 0 && distance < SAFETY_DISTANCE){
      safe = false;
    }
  }
  return safe;
}

bool Road::free_lane(Vehicle& car, LANE lane){
  std::vector<Vehicle> target_lane = get_lane_status(lane);
  bool is_free = true;
  for (int i = 0; i < target_lane.size(); i++) {
    double distance = std::abs(target_lane[i].get_s() - car.get_s());
    if(distance < GUARD_DISTANCE){
      is_free = false;
    }
  }
  return is_free;
}

LANE Road::lane_change_available(Vehicle& car){
  LANE car_lane = car.lane();
  LANE target_lane = car_lane;

  if (car_lane == LANE::LEFT || car_lane == LANE::RIGHT) {
    if (free_lane(car, LANE::CENTER)) {
      target_lane = LANE::CENTER;
    }
  } else {
    if (free_lane(car, LANE::LEFT)) {
      target_lane = LANE::LEFT;
    } else if (free_lane(car, LANE::RIGHT)) {
      target_lane = LANE::RIGHT;
    }
  }
  return target_lane;
}
