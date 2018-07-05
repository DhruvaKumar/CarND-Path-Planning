# Path planning

The goal of the project is to plan and navigate a highway with other traffic in simulation. The speed limit is 50 mph. The rest of the traffic is driving at +-10 mph of the speed limit. The [simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2) provides the car's state and states of other vehicles (retreived from localization and sensor fusion modules). We are also provided the map of the highway along with sparse reference waypoints. The objective is to drive efficiently, within the speed limit, without colliding with other vehicles. The car should not exceed a total accerleration of 10 m/s^2 and jerk greater than 10 m/s^3. This project was done as a part of Udacity's Self-Driving Car Engineer Nanodegree Program. The original repo can be found [here](https://github.com/udacity/CarND-Path-Planning-Project).

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
* [simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`


## Data

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop. The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

Data from the simulator:

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


## Implementation

### Trajectory generation
I used a [cubic spline generator](http://kluge.in-chemnitz.de/opensource/spline/) to generate waypoints at the desired acceleration and velocity along the reference waypoints. The simulator uses a perfect controller and will visit the waypoints it receives every .02 seconds. To generate smooth trajectories, I used the previous waypoints provided from the simulator and appended new waypoints according to the constraints and planner decisions. The downside of using the previous waypoints is that the actuation is not as reactive is it can be. The planner provides the goal lane which the trajectory generator uses. 

### Prediction
This module uses the sensor fusion data of other vehicles and depending on their location and velocity, it predicts where they'll be after a certain time frame. It lets the planner know if there is a vehicle ahead, on the left or on the right.

### Behavioral planner
The planner uses the prediction data to provide the target goal lane and acceleration to the trajectory generator module. Right now, the planner is simple. The ego car will keep its lane until there's a vehicle in front. If so, it'll try to change lanes. If other vehicles are blocking adjacent lanes, it'll deaccelerate to the velocity of the vehicle ahead and follow it. When it finds a suitable window in valid adjacent lanes, it'll accelerate and change lanes.

## Result

<!-- <a href="http://www.youtube.com/watch?feature=player_embedded&v=hj0-zy3l8v8
" target="_blank"><img src="http://img.youtube.com/vi/hj0-zy3l8v8/0.jpg" 
alt="mpc" width="320" height="220" border="10" /></a> -->


## Thoughts and improvement
The current implementation is simple and definitely has scope for improvement. Currently, the planner does a greedy search of choosing when to change lanes. If there's a slow vehicle in front, it'll look within a 30m window in the adjacent lanes. It'll choose whichever empty window it finds first. This might not necessarily be optimum. By looking ahead, one can predict the average lane speeds and thus choose a lane which is faster even if the ego car has to wait in its current lane for some time. If the planner has all lanes available to choose from, it can then default to the middle lane so that it maximizes its options in the future for changing lanes. By defining multiple objectives and thus cost functions, the task would then be choosing which cost functions best describes the goal of the planner and tuning the weights of the individual cost functions such that the planner works well in various scenarios.


