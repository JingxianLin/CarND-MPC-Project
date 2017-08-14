# CarND-Controls-MPC
My implementation of MPC Project for Self-Driving Car Engineer Nanodegree Program is included here.  This file is based on that provided by Udacity and extended with the reflection part at the end.

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).



## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Reflection

The global kinematic model includes the state vector [x, y, psi, v, cte, epsi], actuators [delta, a], and update equations:

x1 = x0 + v0 * cos(psi0) * dt
y1 = y0 + v0 * sin(psi0) * dt
psi1 = psi0 + v0 * delta0 / Lf * dt
v1 = v0 + a0 * dt
cte1 = (f0 - y0) + (v0 * sin(epsi0) * dt)
epsi1 = (psi0 - psides0) + (v0 * delta0 / Lf * dt)

where f0 = polyeval(coeffs, x0), psides0 = atan(f'(x0)), and dt (timestep frequency) is set by first determining a reasonable range for T (prediction horizon).  One small value of 1.25 seconds (N = 25, dt = 0.05) for T is tried at the beginning, and the car dives into the lake, so T is increased to 1.5 seconds (N = 15, dt = 0.1), but the car drives off the track around the sharp corner.  Then dt and N are tuned to 0.15 and 10, respectively, and the vehicle can successfully drive a lap.  Prior to the MPC procedure, waypoints are transformed from the map's coordinate system into the car's coordinate system, so the vehicle state is simplified as [0, 0, 0, v, polyeval(coeffs, 0), -atan(coeffs[1])].  Tuning these parameters can get a better understanding of the Model Predictive Controller.

To compensate for the latency of the controller, first assume the previous actuators don't change in the latency duration.  Then estimate the state vector at the end of the latency time period with the intial state [0, 0, 0, v, polyeval(coeffs, 0), -atan(coeffs[1])] according to the above update equations, for example, x1 = v * latency, y1 = 0, ...  This new vehicle state is applied to the MPC procedure.  After handling latency, the car is able to drive more stably.  Many thanks for reviewer's feedback.