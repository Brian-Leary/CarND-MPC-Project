# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Project Description
###Overview
Model Predictive Control (MPC) is an advanced method of process control which relies on dynamic models of the process. MPC differs from PID controller in that it has the ability to anticipate future events and can take control actions accordingly.

The MPC framework consists of four main components:

* Prediction Horizon (T), or our trajectory, which is the duration of the path we're looking to predict. This is parameterized by a number of time steps N spaced out by a time dt. The number of variables optimized is directly proportional to N, so this must be considered if there are computational limitations to be considered.

* Vehicle Model, which is the set of equations that describes system behavior and updates across time steps. We used a simplified kinematic model (bicycle model) described by a state with six parameters:
  * x: car's position on the x-axis
  * y: car's position on the y axis
  * psi: car's heading/direction
  * v: car's velocity
  * cte: cross track error (car's distance from desired path)
  * epsi: orientation error (difference in car's heading vs desired heading)
                 
  Vehicle model equations are implemented in lines 139-144 in main.cpp and updated in lines 146-151 in MPC.cpp.

* Constraints necessary to run the model in a realistic (and hopefully smooth) manner. For example, a car cannot make 90-degree turns so we should construct our model with such limitations in mind.
  * steering angle/delta: restricted to range [-25°, 25°]
  * acceleration: restricted to range [-1, 1] (full brake to full throttle)

* Cost Function, which is optimized and stands as the basis of the whole control process. The cost function is not limited to the state (x, y, psi), which gives us cte and epsi. We include a penalty in the cost function for not maintaining a reference velocity ref_v. We also include control inputs (steering angle, acceleration) so as to penalize the magnitude of the input, as well as the change rate.

  The cost function is implemented in lines 70-90 in MPC.cpp.
  
Timestep Length and Elapsed Duration (N and dt)
N and dt are critical parameters in the optimization process. The product of N and dt gives us the Prediction Horizon (T), which is the window of time for our predictions from the optimization process. Proper tuning requires understanding the following:
  * Ideally, you'd like T (N * dt) to be as large as possible. But in terms of a moving car, it wouldn't make sense to have T be any longer than a few seconds. Beyond that, conditions have changed too much for predictions to be reliable.
  * A high dt results in less frequent actuations, which leads to difficulty in keeping up with our reference trajectory. This is known as discretization error, so in an effort to minimize this, we'd like as small a figure as possible for dt.
  * If we'd like T to be large but dt to be small, that seems to indicate that we're obligated to inflate N as much as possible. In doing so, one should keep in mind that N determines the length of the control inputs vector to be fed into our MPC, i.e. the number of variables that will need to be optimized. This means the higher N gets, the higher the computational cost involved, as previously mentioned.

N (10) and dt (0.1) were chosen for this project with the information above in mind, through trial and error results with the simulator and with some need for pragmatism involved, given the limitations of the hardware I have available to me. Notes on results from different variations are included in the code (MPC.cpp).
 
Coordinate Systems
The server returns waypoints using the global coordinate system, which is different from the car's coordinate system. These are transformed in lines 104-111 in main.cpp to make it easier to display them and to calculate cte and epsi values.

Reference Values and Weightings
The goal I had in mind was to see a top speed of 100mph reached whilst not leaving the track. A reference value for velocity (ref_v) of 130, with minimal weighting applied to velocity but significant weightings applied to cte, epsi and delta (1, 1500, 2000 and 1000, respectively) just barely achieved this goal. This can be viewed in lines 30-91 in MPC.cpp.

Model Predictive Control with Latency
A real car would have a delay in the time between actuations being determined and when they get executed. Considering this, a 100 millisecond delay has been implemented in line 223 of main.cpp, prior to sending data to the simulator. To counter this, the state has been predicted one step ahead, before feeding it to the solver: lines 133-151.


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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

