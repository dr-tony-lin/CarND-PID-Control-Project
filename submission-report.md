# PID Control Project
[Convergence]: ./convergence.png
[PID]: ./pid.png
[MA]: ./ma.png
[Mean Angle]: ./cm-angle.png
[Mean Turn]: ./cm-turn.png
[PID]: ./pid.mp4
[MA]: ./ma.mp4
[Mean Angle]: ./cm-angle.mp4
[Mean Turn]: ./cm-turn.mp4

## Content of the Submission and Usage
This submission includes the following c++ files:
* pid.cpp: the main function that communicates with the simulator and drive the PID process.
* twiddle.cpp: the main twiddle function for narrowing the PID parameters
* control/PID.[h, cpp]: the PID controller
* utils/Reducer.h: the Reducer class for sum, mean, min, max on a collection of samples.
* tune/Twiddle.[h, cpp]: Provides twiddle implementation
* tune/CarTwiddle.[h, cpp]: a subclass of Twiddle for a car model

### Usage
**The PID Controller**
The PID controller can be launched with the following command:

    ./pid [-s kp kd ki] [-v kp kd ki] [-max_speed speed]

Where:

* -s: specifies the PID coefficients for steering. The default is: k<sub>p</sub> = 0.108, k<sub>d</sub> = 3.52, and k<sub>i</sub> = 0
* -v: specifies the PID coefficients for speed. The default is: k<sub>p</sub> = 13.5795, k<sub>d</sub>= -11.4359, and k<sub>i</sub> = 0
* -max_speed, specify the maximal driving speed

The program will listen on port 4567 for an incoming simulator connection. Only one simulator should be connected at anytime, though the program does not prohibit it. To start a new simulator, terminate the existing one first, then start a new one.

To start the simulator:

#### On Windows

    term2_sim9.exe

#### Other platforms:

    ./term2_sim9

**Launch Twiddle**
Twiddle can be launched with:

    twiddle [-accel] [-steps steps] [-dt dt] [-y y] [-len length] [-target target] [-speed speed] [-drift drift]

Where:

* -accel: run twiddle for acceleration/deceleration coefficients, default is to run twiddle for steering coefficients
* -steps: number of steps to move the vehicle in every run, default is 100
* -dt: delta time for each step, default is 0.1
* -y: the y coordinate of the vehicle, default is 1
* -len: length of the vehicle between the front wheels and the center of mass, default is 2.5 (m)
* -target: the target value to reach, default is 0
* -speed: speed of the vehicle to simulate
* drift: steering drift, default is 0

#### Build
For Windows, Bash on Ubuntu on Windows should be used. Both gcc and clang can be used to build the program.

To build the program, invoke the following commands on the bash terminal:
```
mkdir build
cd build
cmake ..
make
```

The program can be built to output more information for disgnosis purposes by defining **VERBOSE_OUT** macro.
In addition, the following macros can be defined:

* PLOT_WITH_MATPLOT: when defined on Mac, twiddle will use python matplotlib module to plot the PID convergence diagram. In order to do this, tpython 2.7 with numpy, and matplotlib are required. Furthermore, on Bash on Windows, an X11 server is required and the DISPLAY environment need to be set accordingly.

* STABILIZE_MOTION: when defined, the program will try to stabilize the vehicle. WHen this is defined, USE_MEAN_TURN can also be defined to use moving average of turns. Please refer to the **Steering** section. For example: 

    cmake -DSTABILIZE_MOTION=1 -DUSE_MEAN_TURN=1 ..

#### Build API Documentation
The documentation for functions, classes and methods are included in the header files in Doxygen format. To generate Api documentation with the included doxygen.cfg:

1. Make sure doxygen is installed
2. cd to the root folder of this project
3. Issue the following command:

    doxygen doxygen.cfg

## The Implementation

### The source code structure
The src folder contains main.cpp, twiddle.cpp, and three folders:
. tune: contains twiddle implementation files.
. control: contains the PID control class
. utils: contains the Reducer class

##### File names
In this project, I made a class to have its own .h, and .cpp files. More specifically, a class source file contains exactly one class, and has the same name as the class it contains.

#### The libs folder
The libs folder contains Eigen, json.hpp, and matplotlibcpp.h.

## Twiddle class
The class implements twiddle algorithm in **twiddle()** method. Its subclass is required to implement the model simulation in the **run()** method.

## CarTwiddle class
This class implements a simple vehicle motion model that simulates the coordinate and yaw of a vehicle from the steering, velocity, acceleration and delta time. It is a subclass of Twiddle, and implements the **run()** method. The motion model is implements in **move()** method.

## PID class
This class implements PID controller. The **updateError()** is used to update the error, then the control value can be obtained from **getControl()** method. In addition to updating error, one can also update the value using **updateValue()** method, and the error will be computed from the target that can be set using **setTarget()** method.

## Reducer class
This class provides aggregation for a set of samples. This includes mean, weighted mean, sum, max, and min.

## Determine PID coefficients
Two PID controllers are used in this project:

* Steering PID controller: for controlling the steering so the car is driven as closed to the center as possible.
* Acceleration PID control: for controlling the acceleration and deceleration of vehicle. The project aimed to make the vehicle fast and furious. It decides the desirable speed according to the steering angle. Then the PIC controller is used to obtain the acceleration or deceleration.

The twiddle program was used to help to narrow down the range of PID coefficients to reduce the efforts of finding optimal coefficients.

For the acceleration PID coefficients, the values produced by twiddle are used without further tuning. They seem working fine for the purpose.

The steering PID coefficients, on the other hand, required a lot of tuning efforts to reduce oscillation, and produce reasonable controller outputs.

The convergence diagram of different PID coefficients are shown in [this diagram](./convergence.png). The tuning process started with the coefficients returned from twiddle as follows:

1. If the vehicle has difficulties to stay on the road at turns, increase k<sub>p</sub>.
2. Otherwise, if the vehicle oscillates a lot, reduce k<sub>p</sub>
3. Fine tune k<sub>d</sub>, if the vehicle shows lots of sharp movements, reduce the value, otherwise, increase the value if this yield better motion.

The default PID coefficients that are chosen for this project are:

k<sub>p</sub> = 0.108, k<sub>d</sub> = 3.52, and k<sub>i</sub> = 0 for steering

and

k<sub>p</sub> = 13.5795, k<sub>d</sub>= -11.4359, and k<sub>i</sub> = 0 for aceleration. 

The k<sub>i</sub> in both controllers are 0 as no drift was assumed, and they seem fine with the simulator.

## Steering
The CTE values reported the simulator are updated to the steering PID controller. The steering control value is then obtained from the PID controller, then normalized to [-1 1] range.

The simulation shows large oscillation on curved road due to the following reasons:

1. The PID process
2. When the vehicle is moved back to the center of the road, 0 steering will be sent back to the simulator. However, this should not be the case for curved road. Instead, the vehicle should be steered according to the curvature. This, unfortunately, cannot be obtained from the feedbacks provided by the simulator.

To reduce oscillation, I have tried two approaches:

1. Use moving average of the steering control values to smooth out steering
2. Try to replace small spike in CTE with fewer wider CTE peaks.

### Moving average
In this approach, every steering control values generated by the PID controllerto a reducer, and let the reducer compute weighted average of the values within a kernel of some certain size (5 currently).

### Reduce CTE Spikes
Could we reduce CTE spikes by widen it so the car will oscillate less while still can manage to stay on the road? On approach that I have tried is to remove some portion of steering as follows:
1. Take the moving average of the past 20 to 40 angles or turns (30 in my case).
3. Take the moving average of the past 3 to 7 steerings (5 in my case), and substract it with the moving average from 1. This effectively applies two different filters on the steering, one low pass, and another high pass.

## Throttling
The vehicle speed is set according to the steering angle, the bigger the angle, the slower the speed should be for the vehicle to stay on the course. The **computeSpeedTarget()** function computes the fastest possible speed for the vehicle.

The difference between the target speed and the current speed reading is fed into the acceleration PID controller. The output of the controller is the acceleration or deceleration for the vehicle.

The acceleration/deceleration is clamped to range [-20, 8], and then converted to a throttle value by the **computeThrottl()** function. 

For acceleration, a minimal throttle value is required to keep the vehicle at the given speed, and extra throttle that is required to accelerate the vehicle is added.

For deceleration, the throttle will be set to a negative value, that is to brake the vehicle.

It also needs to be noted that the algorithm for computing the throttle from acceleration or deceleration is purely empirical. This is just like real world driving where we, at least for most people, will not know how fast the velicle will accelerate or decelerate when we step on the pedal or brake. We simply do until we have reached the target, and we step harder when we need things to happen faster.

## Results
The following table shows the simulation result with different steering algorithms, including the videos recording during the simulations, the charts that show the curves of the speed, original steering, adjusted steering, mean, and CTE of the vehicle from the starting point to the bridge. The tuned steering PID coefficients used for each algorithm are also shown.

|                            |       Video         |       Chart           |    PID Coefficients     |
|:---------------------------|:-------------------:|:---------------------:|:-----------------------:|
| PID Only                   |[Video](pid.mp4)     | [Chart](pid.png)      | kp=0.108, kd=3.52, ki=0 |
| Weighted Moving Average    |[Video](ma.mp4)      | [Chart](ma.png)       | kp=0.075, kd=2.5, ki=0  |
| Reduction using Mean Angle |[Video](cm-angle.mp4)| [Chart](cm-angle.png) | kp=0.13, kd=4, ki=0     |
| Reduction using Mean Turn  |[Video](cm-turn.mp4) | [Chart](cm-turn.png)  | kp=0.13, kd=4, ki=0     |

From the chart, we can observe the followings:

1. Moving average along did not reduce but increased the oscillation
2. By comparing the CTE curves on the charts, oscillation reduction using moving average did have reduced the oscillation, and was able to achieve a higher speed (up to **89Mph** has been observed).
3. Mean angle performed better than mean turn which had resulted in larger CTE. 
4. Also observed in my experiments, oscillation reduction without using mean steering, and clamping changes in steering value may cause the vehicle to oscillate more.

### Discussion
The above algorithms require different PID coefficients to reach certain stability. This, of course, might affect the characteristics of the performance curve shown in the above charts. But for being able to run the simulator at a higher speed with reduced oscillation, I think I have achieved the goal.

The extra works that I have done to stabilize the vehicle was to due a challenge from one of my friend in that, with PID alone, it looked like the driver was drunk! I think the driver is less drunk with the oscillation reduction apparoch, or maybe just for fun!
