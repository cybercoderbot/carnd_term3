#include "planner.h"

std::string strLane(LANE lane){
  if (lane == LANE::LEFT){
    return "LEFT";
  } else if(lane == LANE::CENTER){
    return "CENTER";
  } else{
    return "RIGHT";
  }
}

std::string strState(STATE s){
  if (s == STATE::KEEP_LANE){
    return "KEEP_LANE";
  } else if(s == STATE::CHANGE_LEFT){
    return "CHANGE_LEFT";
  } else if(s == STATE::START){
    return "START";
  } else{
    return "CHANGE_RIGHT";
  }
}

LANE getLane(double d){
  LANE lane;
  if (d < 4.0) {
    lane = LANE::LEFT;
  }
  else if ((d >= 4.0) && (d < 8.0)) {
    lane = LANE::CENTER;
  }
  else {
    lane = LANE::RIGHT;
  }
  return lane;
}

double getLaneD(LANE lane){
  double d;
  if (lane == LANE::LEFT) {
    d = 2.0;
  }
  else if (lane == LANE::CENTER) {
    d = 6.0;
  }
  else {
    d = 10.0;
  }
  return d;
}

double getLaneD(double D){
  double d;
  if (D < 4.0) {
    d = 2.0;
  }
  else if ((D >= 4.0) && (D < 8.0)) {
    d = 6.0;
  }
  else {
    d = 10.0;
  }
  return d;
}

Planner::Planner()
  : state_ (STATE::START)
{
}


/* JMT*/
std::vector<double> Planner::JMT(std::vector<double> start, std::vector <double> end, double T) {
  /*
  Calculate the Jerk Minimizing Trajectory that connects the initial state
  to the final state in time T.

  INPUTS

  start - the vehicles start location given as a length three array
      corresponding to initial values of [s, s_dot, s_double_dot]

  end   - the desired end state for vehicle. Like "start" this is a
      length three array.

  T     - The duration, in seconds, over which this maneuver should occur.

  OUTPUT
  an array of length 6, each value corresponding to a coefficent in the polynomial
  s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
  */

  // prepare matrix A with coefficents
  Eigen::MatrixXd A(3,3);
  Eigen::MatrixXd B(3,1);

  A << T*T*T, T*T*T*T, T*T*T*T*T,
       3*T*T, 4*T*T*T,5*T*T*T*T,
       6*T, 12*T*T, 20*T*T*T;

  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
       end[1]-(start[1]+start[2]*T),
       end[2]-start[2];

  Eigen::MatrixXd Ai = A.inverse();
  Eigen::MatrixXd C = Ai*B;

  return {start[0], start[1], .5*start[2], C.data()[0], C.data()[1], C.data()[2]};
}

void Planner::updateTrajectory(Map& map, std::vector<std::vector<double>>& trajectory){

  // jmt
  double T = n_ * AT;
  std::vector<double> poly_s = JMT(start_s_, end_s_, T);
  std::vector<double> poly_d = JMT(start_d_, end_d_, T);

  double t, next_s, next_d, mod_s, mod_d;
  std::vector <double> XY;
  for(int i = 0; i < n_; i++) {

    t = AT*i;

    next_s = 0.0;
    next_d = 0.0;
    for (int a = 0; a < poly_s.size(); a++) {
      next_s += poly_s[a] * pow(t, a);
      next_d += poly_d[a] * pow(t, a);
    }
    mod_s = fmod(next_s, TRACK_DISTANCE);
    mod_d = fmod(next_d, ROAD_WIDTH);

    XY = map.getXY(mod_s, mod_d);

    trajectory[0].push_back(XY[0]);
    trajectory[1].push_back(XY[1]);
  }

}

void Planner::createTrajectory(Map& map, Road& road, Vehicle& car, 
                                std::vector<std::vector<double>>& trajectory) {

  std::cout << "STATE: " << strState(state_) << std::endl;
  std::cout << "LANE: " << strLane(car.lane()) << std::endl;

  int current_points = trajectory[0].size();
  new_points_ = false;

  if (current_points < POINTS) {
    new_points_ = true;

    // first trajectory
    if (state_ == STATE::START) {

      startCar(car);

    } else if (state_ == STATE::KEEP_LANE) {

      // FREE LANE
      if (road.isSafeLane(car, car.lane())) {
        stayInLane(car);
      // LANE CHANGE NEEDED
      } else {
        LANE target_lane = road.laneChangeAvailable(car);
        if (target_lane == car.lane()){
          // not possible -> reduce speed
          reduceSpeed(car);
        } else {
          changeLane(car, target_lane);
        }
      }
    } else {
      LANE new_lane = getLane(car.prevD()[0]);
      if(road.isSafeLane(car, new_lane)){
        stayInLane(car);
      } else {
        // not possible -> reduce speed
        reduceSpeed(car);
      }
    }
  }

  // have we generated new points?
  if (new_points_) {
    updateTrajectory(map, trajectory);
  }

}

/* FSM */
void Planner::setState(LANE current_lane, LANE target_lane){
  if (current_lane == target_lane){
    state_ = STATE::KEEP_LANE;
  } else {
    // not equal
    if(current_lane == LANE::LEFT){
      state_ = STATE::CHANGE_RIGHT;
    } else if(current_lane == LANE::RIGHT){
      state_ = STATE::CHANGE_LEFT;
    } else {
      if(target_lane == LANE::LEFT){
        state_ = STATE::CHANGE_LEFT;
      } else {
        state_ = STATE::CHANGE_RIGHT;
      }
    }
  }
}

/* APPLY ACTION */
void Planner::applyAction(Vehicle& car, LANE current_lane, LANE target_lane){
  car.setPrevS(end_s_);
  car.setPrevD(end_d_);
  setState(current_lane, target_lane);
}

/* ACTIONS */
void Planner::startCar(Vehicle& car){
  std::cout << "ACTION: start_car" << std::endl;
  n_ = 4*POINTS; // 4 cycles to start
  double target_v = SPEED_LIMIT*0.5;
  double target_s = car.getS() + n_ * AT * target_v;;

  start_s_ = {car.getS(), car.getV(), 0.0};
  end_s_= {target_s, target_v, 0.0};

  start_d_ = {getLaneD(car.lane()), 0.0, 0.0};
  end_d_ = {getLaneD(car.lane()), 0.0, 0.0};

  applyAction(car, car.lane(), car.lane());
}

void Planner::stayInLane(Vehicle& car){
  std::cout << "ACTION: stay_in_lane" << std::endl;
  n_ = CYCLES*POINTS;
  double target_v = min(car.prevS()[1]*1.3, SPEED_LIMIT);
  double target_s = car.prevS()[0] + n_ * AT * target_v;

  start_s_ = {car.prevS()[0], car.prevS()[1], car.prevS()[2]};
  end_s_ = {target_s, target_v, 0.0};

  double target_d = getLaneD(car.prevD()[0]);

  start_d_ = {getLaneD(car.prevD()[0]), 0.0, 0.0};
  end_d_ = {target_d, 0.0, 0.0};

  applyAction(car, getLane(car.prevD()[0]), getLane(car.prevD()[0]));
}

void Planner::reduceSpeed(Vehicle& car){
  std::cout << "ACTION: reduce_speed" << std::endl;
  n_ = CYCLES*POINTS;
  new_points_ = true;
  double target_v = max(car.prevS()[1]*0.8, SPEED_LIMIT/2);
  double target_s = car.prevS()[0] + n_ * AT * target_v;

  start_s_ = {car.prevS()[0], car.prevS()[1], car.prevS()[2]};
  end_s_ = {target_s, target_v, 0.0};

  double target_d = getLaneD(car.prevD()[0]);

  start_d_ = {getLaneD(car.prevD()[0]), 0.0, 0.0};
  end_d_ = {target_d, 0.0, 0.0};

  applyAction(car, getLane(car.prevD()[0]), getLane(car.prevD()[0]));
}

void Planner::changeLane(Vehicle& car, LANE target_lane){
  std::cout << "ACTION: reduce_speed" << std::endl;
  n_ = CYCLES*POINTS;
  new_points_ = true;
  double target_v = car.prevS()[1];
  double target_s = car.prevS()[0] + n_ * AT * target_v;

  start_s_ = {car.prevS()[0], car.prevS()[1], car.prevS()[2]};
  end_s_ = {target_s, target_v, 0.0};

  double target_d = getLaneD(target_lane);

  start_d_ = {getLaneD(car.prevD()[0]), 0.0, 0.0};
  end_d_ = {target_d, 0.0, 0.0};

  applyAction(car, getLane(car.prevD()[0]), getLane(target_d));
}
