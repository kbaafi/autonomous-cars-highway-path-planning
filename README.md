# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Simulator. You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

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

The car uses a perfect controller and will visit every (x,y) point it receives in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car.

## Resources

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single header file is really easy to use.

# Behavioral Path Planning for Self-Driving cars
  See `EgoCar::drive`
  This project attempts to provide a safe way of maneuvering between a controlled car and other independent cars or driving agents on a predefined path which in this case is a three lane highway in a simulator environment.

  At each timestep, the simulator provides the location and velocity statistics of each car on the road. In controlling the car, the following steps are taken.
  1. The suitability of each lane is first examined using a set of cost functions which denote the cost of moving into that lane. The most ideal lane is the lane with the minimum cost.
  2. When a minimum cost lane is found, a counter counts the number of times said lane has been considered as the optimal cost.
  3. After a given threshold the optimal lane is considered as good enough to be changed to. This threshold is an effective way of ensuring that the car does not exhibit erratic driving behavior.
  4. Before changing into the target lane, the target lane is examined for potential collisions. If a collision can potentially occur, the counter is reset and the process is started all over again. Starting the process again emphasizes safety over the 'need for speed'
  5. Finally the car makes sure that it does not collide with other cars on the road

## Finding the Minimum Cost lane
  See `EgoCar::rank_lanes`
  In finding the minimum cost lane, each lane is examined by considering the space ahead of the current car. The following metrics are considered
    1. Distance cost: cost based on the distance between the controlled car and the closest car ahead. This is given by
      `distance cost = weight_distance *(((max_obs_distance-closest_distance)/max_obs_distance))`
      where `max_obs_distance` denotes how far ahead the car looks, `weight_distance` denotes how much importance should be placed on this cost and `closest_distance` refers to the actual distance between the controlled car and the closest car ahead in that lane.
      See function `EgoCar::distance_cost` in `egocar.cpp`
    2. Velocity cost: cost based on the velocity of the closest car ahead.
      Using the closest car ahead in the lane, the cost due to the velocity of the car ahead is given by
      `velocity cost = weight_velocity*(|_max_velocity - velocity of closest car|/_max_velocity))`
      See function `EgoCar::velocity_cost_ahead` in `egocar.cpp`
    3. Lane Change cost: the cost related to moving the car into the next lane. This is given by
      `lane change cost = |destination lane - current lane|/(number of lanes-1)`
      See function `EgoCar::lane_change_cost` in `egocar.cpp`
  The lane with the total minimum cost is considered as the optimal cost lane.

### A more robust minimum cost lane
  A safer minimum cost lane can be found by setting a baseline that looks ahead from a distance slightly behind the car. This way cars directly adjacent to and slightly behind the self-driving car are taken into consideration. This may lead a less responsive car in thick traffic but on the other hand produces a much safer driving experience.

## Lane changes
  After a preset threshold,the car may move only after checking to see if the lane change will not cause a collision. If a lane change is likely to occur, the whole search for a minimum cost lane is restarted.
  If a collision is not detected, the car makes a move into the optimal lane
### Double Lane changes
  In the case where the optimal lane is more than one lane away, the desired lane is the lane next to the current lane in the direction of the optimal lane. After the threshold is exceeded, the lane change is effected if there is no chance of collision. After a lane change, the self-driving car is forced to stay in the lane for a while before attempting the next lane change.

## Collision detection and avoidance
  See `EgoCar::is_collision_free`
  The following measures were used to detect collision before a lane change:
  1. The closest car ahead in the current lane must be beyond a certain distance ahead
  2. The closest car ahead in the target lane must be ahead of the self-driving car by a distance. Also it cannot be beside the self-driving car.
  3. The closest car behind in the target lane must be a certain distance behind the car
  4. The speed of the car behind in the target lane must be 15mph slower than the speed of the self-driving car
  5. The speed of the self-driving car must be more than 30mph before a lane change can occur

## Trajectory Generation
  In order to drive smoothly the self-driving car needs a set of waypoints that it can follow such that changes in acceleration and velocity do not produce erratic or unsafe driving. The Trajectory generation is facilitated by use of a cubic spline. The choice of the points that the spline must pass through must be such that the tangent of the inflexion is a low as possible and curve trajectory just eases into the desired position in the target lane.
  The first two points of the spline are taken from the last two waypoints in the previous cycle. The next waypoints are picked at x = 50, x = 80 and x = 90. The corresponding y values are calculated and the resulting set of points are fed to the spline function which produces a smooth, jerk free trajectory.

## Emergency collisions
  In the case where a car is getting too close, the following rules apply:
  * If there is another car behind the self-driving, the self-driving car matches the speed of that car if its speed is less than that of the self-driving
  * If there is no car behind the self-driving car, the car attempts to come to a complete stop at a high deceleration rate. 



[See here](https://www.youtube.com/watch?v=4bjPGdgcmnQ)
