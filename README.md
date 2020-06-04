# PID Cntroller
This project implements a PID Controller in C++ to maneuver a vehicle around a track in the simulator.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## PID Controller parameters

#### Proportional (P)
This parameter controls the error proportionally, so the control signal increases as this parameter value is increased. However, this results in increasing oscillations, as this parameter causes the car to steer proportional but opposite to the cross-track error.

#### Integral (I)
This parameter integrates, i.e. accumulates the error. This helps in reducing the steady state error. This parameter also helps increase the control signal in case there is a bias, and therefore helps driving the bias error down.

#### Derivative (D)
This parameter helps to add damping and decreases overshoot, as it controls the rate of change of error. It counteracts the P component’s overshoot, and helps the car to approach the center line smoother

## Tuning of parameters
Tuning of parameters was done manually to understand the effects of each parameter. A second PID controller (throttle_pid) was added to control the speed.

I started with setting Kp = 0.05 and setting Ki = 0 and Kd = 0. This caused a lot of oscillations, as expected, due to the P component.
Setting Kd to a large value i.e. > 1 helped reduce overshoot and reduce oscillations. Setting it even larger i.e. > 2 was the optimum value that resulted in stability.
Ki had a small effect, however it controls the bias, and this helps around turns. This only needs to be a small value, as a larger value adds to overshoot and settling time.
These parameters Kp, Ki and Kd for Proportional, Integral and Derivative can be optimized using a Twiddle algorithm.

Here are final values of parameters for both the PID controllers
|              | Kp   | Ki    | Kd   |
|--------------|------|-------|------|
| **Steering** | 0.06 | 0.004 | 2.15 |
| **Speed**    | 0.07 | 0.002 | 2.47 |
