#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include "utils.h"

using namespace std;

class Vehicle {

  /* ["sensor_fusion"] A 2d vector of cars and then that car's [
  id: car's unique ID,
  x: car's x position in map coordinates,
  y: car's y position in map coordinates,
  v: car's speed,
  s: car's s position in frenet coordinates,
  d: car's d position in frenet coordinates. */

  protected:

    int id_;
    double x_;
    double y_;
    double v_;
    double s_;
    double d_;
    double yaw_;

    std::vector<double> previous_s_;
    std::vector<double> previous_d_;

  public:

    Vehicle();
    Vehicle(int id, double x, double y, double v, double s, double d);
    ~Vehicle(){};

    // int getId();
    double getV();
    double getS();
    LANE lane();

    void update(double x, double y, double v, double s, double d, double yaw);
    void setPrevS(std::vector<double> pre_s);
    void setPrevD(std::vector<double> pre_d);
    std::vector<double> prevS();
    std::vector<double> prevD();

};

#endif // VEHICLE_H
