# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
> https://youtu.be/eFdlfFKR01U

## Obtain the data:
First we obtain the main car's localization Data: 
Previous path data given to the Planner, Previous path's end s and d values, the Sensor Fusion Data which is a list of all other cars on the same side of the road.

## Current road status?
Next we define a path made up of (x,y) points that the car will visit sequentially every .02 seconds. We need check for a previous path and update the car's current s-coordinate. A variable for the reference velocity will be used to dictate to the main car whether or not to accelerate (i.e. the target velocity it should be at). Next we iterate throught the sensor_fusion parameter (the list of data for the other vehicles nearby) to assess the main car's surroundings and perform our control logic. We us the lateral coordinate d of the other vehicle to determine it's current lane. Now we can check if the car in same lane or about to change to the same lane. We also estimate the location of the car in s-d coordinates in the next future step and check whether the car will be occupying the same lane or not.

## Now that we know? should we do anything?
If the car is in the same lane we check whether or not it is both ahead and within a safety distance of the main car calculated based on the main car's speed. If the previous condition is true then set the too_close flag to positive and record the id and speed of the car ahead. However, if it doesn't occupy the same exact lane check if it is next to and ahead of the car by a turning-safety distance again based on the speed. If this previous condition is true, check whether it is staying in the adjacent lane or entering the current lane to prevent unsafe turns. Based on the last two safety conditions we set the safe_left and safe_right flags accordingly to prevent unsafe turns.

### For debugging logic behaviour continuously output the status of the three main behaviour-changing flags to the console.

## Don't panick! this is what you should do:

Based on the above mentioned flags we now need to perform any needed behaviour changes. 

is the car ahead too close?
* YES
	can you change to a lane?
	* YES
		is it safe to change to left or right? (without a cost function at the moment, inital preference is given to going left)
  		* LEFT
  			Change lanes
  		* RIGHT
  			Change lanes
  	* NO
		Decelerate according to speed and distance
* NO
	accelerate towards speed limit

## Where do I go though...?

Next we need to build the path for the car to move onto now that the speed and lane has been decided. First we initialise vectors to store the x-y map points, currently the origin for calculations are the car's current global x, y, and yaw values. We check the size of the precious path and either initialise or extract points from the previous path to add to the x-y map points. The next step is to get the x-y coordinates for distances 30, 60 and 90 meters ahead of the car current position in s cooredinates. Next we Iterate throught the all the points in x-y vectors and shift from the map-xy coordinates to the car-xy coordinates for easier calculations. All we need now is to initialise a spline element s and set the points for this spline using our x-y points that we calculated so far. For our final step we initialise vectors to store the future set of x and y values, we iterate through the values of the previous path and add them to the future path vectors. After getting the spline setup for the next 90 m in s-coordinates we select a target range to immediately follow and draw in the simulator. Using the spline we iterate through our transformed future x values to get the corresponding y values. The vectors for future x and y values should always have 50 elements, we can achieve this by checking on the previous path size and subtracting from 50. make sure in each iteration that the values are from the previous calculated position and not the current position.



### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

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

## Tips

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
