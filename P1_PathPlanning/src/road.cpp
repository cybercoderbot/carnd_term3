#include "road.h"


void Road::update(std::vector<Vehicle> l, std::vector<Vehicle> c, std::vector<Vehicle> r){
  left_lane_ = l;
  center_lane_ = c;
  right_lane_ = r;
}

std::vector<Vehicle> Road::getLaneStatus(LANE lane){
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

bool Road::isSafeLane(Vehicle& car, LANE lane){
  std::vector<Vehicle> r_car_lane = getLaneStatus(lane);
  bool is_safe = true;
  for (int i = 0; i < r_car_lane.size(); i++) {
    double distance = r_car_lane[i].getS() - car.getS();
    if(distance > 0 && distance < SAFETY_DISTANCE){
      is_safe = false;
    }
  }
  return is_safe;
}

bool Road::isFreeLane(Vehicle& car, LANE lane){
  std::vector<Vehicle> target_lane = getLaneStatus(lane);
  bool is_free = true;
  for (int i = 0; i < target_lane.size(); i++) {
    double distance = std::abs(target_lane[i].getS() - car.getS());
    if(distance < GUARD_DISTANCE){
      is_free = false;
    }
  }
  return is_free;
}

LANE Road::laneChangeAvailable(Vehicle& car){
  LANE car_lane = car.lane();
  LANE target_lane = car_lane;

  if (car_lane == LANE::LEFT || car_lane == LANE::RIGHT) {
    if (isFreeLane(car, LANE::CENTER)) {
      target_lane = LANE::CENTER;
    }
  } else {
    if (isFreeLane(car, LANE::LEFT)) {
      target_lane = LANE::LEFT;
    } else if (isFreeLane(car, LANE::RIGHT)) {
      target_lane = LANE::RIGHT;
    }
  }
  return target_lane;
}
