#include "vehicle.h"

using namespace std;

Vehicle::Vehicle()
  : id_ (-1)
{
}

Vehicle::Vehicle(int id, double x, double y, double v, double s, double d)
  : id_ (id)
  , x_ (x)
  , y_ (y)
  , v_ (v)
  , s_ (s)
  , d_ (d)
{
}

/* GETTERS*/
double Vehicle::get_v(){
  return v_;
}

double Vehicle::get_s(){
  return s_;
}

LANE Vehicle::lane(){
  LANE lane;
  if (d_ < 4.0) {
    lane = LANE::LEFT;
  }
  else if ((d_ >= 4.0) && (d_ < 8.0)) {
    lane = LANE::CENTER;
  }
  else {
    lane = LANE::RIGHT;
  }
  return lane;
}

void Vehicle::update_vehicle_values(double x, double y, double v, double s, double d, double yaw)
{
  x_ = x;
  y_ = y;
  v_ = v;
  s_ = s;
  d_ = d;
  yaw_ = yaw;
}

void Vehicle::set_previous_s(vector<double> pre_s){
  previous_s_ = pre_s;
}

void Vehicle::set_previous_d(vector<double> pre_d){
  previous_d_ = pre_d;
}

vector<double> Vehicle::prev_s(){
  return previous_s_;
}

vector<double> Vehicle::prev_d(){
  return previous_d_;
}
