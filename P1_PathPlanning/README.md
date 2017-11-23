# Project 1: Path Planning
   

### Goals

The goal of this project is to build a path planner that creates smooth, safe trajectories for the car to follow. The highway track has other vehicles, all going different speeds, but approximately obeying the 50 mph ( +/-10 mph) speed limit.

The car transmits its location, along with its sensor fusion data, which estimates the location of all the vehicles on the same side of the road.
The car's localization, sensor fusion data and a sparse map list of waypoints around the highway are provided. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.


The project can be divided into following 3 parts:


## Smoothing Waypoints

The track is approximately 7 km and only has 181 waypoints.  The simulator appear to use linear
interpolation between the waypoints to compute map coordinates from Frenet coordinates.  The track waypoints given in the highway_map.csv file are spaced roughly 30 meters apart, so the first step in the process is to interpolate a set of nearby map waypoints (in the current implementation, five waypoints ahead of and five waypoints behind the ego vehicle are used) and produce a set of much more tightly spaced (0.5 meters apart) waypoints which help to produce more accurate results from the getXY and getFrenet methods and also account for the discontinuity in s values at the end/beginning of the track.


## The Road Map

The spline.h helps us get the Cartesian coordinates for our estimated Frenet points.  This makes implementation the path cycle rather easy by simply taking `fmod` of the total track length in s coordinates in Frenet coordinates. 


## Vehicle Behavior Planning

The telemetry data is used in computing what the next logical move
for the car to make is. A four state Finite State Machine is used to model the car behivior:

 * Start
 * Keep going straight
 * Lane change left
 * Lane change right

In the case of the lane changes, clearly the lane has to be appropriate (can't change 
left out of the leftmost lane) and it has to be acceptable with respect to the 
other cars in the lane.  To both as a cost, if the lane change is not appropriate
the cost is simply set high. In case there is another car on his trajectory it will try to change lanes. If the car's initial position is in the left lane, the car only will change lane to the right. The opposite applies when the car needs to change and it is driving in the right lane.

If the car finds itself in the center of the highway, it will try first, to change to the left lane. If that's not possible and the right lane is safe and free to move in, it will try to change to the right lane.

When driving straight, the car tries to maximize speed with while under the speed limit and not running into the front car.  The case could arise of not being able to change 
lanes, but the car in front of us is going slow, so it is necessary to monitor the 
car in front of us and adjust our speed.

The planner also implements a Jerk Minimizing Trajectory generator in order to create and drive smooth and safe paths when the car needs to change lanes or just to stay in the same lane without spikes on the acceleration nor jerk values. The paths that are fed to the simulator are all segments computed to minimize jerk.  

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Resource

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```


