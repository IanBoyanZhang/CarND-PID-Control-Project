# CarND-Controls-PID

PID
---

This is classical PID control trajectory following project. 

Proportional control is used to set up steering angle control. Proportional Control steers harder when vehicle is far away from
desired trajectory defined by measurement of CTE (Cross Track Error). Roughly, higher gain of proportional control will provide faster 
 system response, lower gain vice versa. By only relying on proportional control, vehicle will continuously overshoot the trajectory instead of following.

To correct overshoot, CTE changing rate could be used, which is described in derivative term. This term can be visulized as resistance of 
 pulling vehicle towards the trajectory.
 
Integral term is introduced to counteract steady state error indicating if we are spending more time on one side
of trajectory than another.

Parameters and Implementation
---

A state machine based twiddle tuner implementation can be found in PID class. The simulator is reset after receiving every
 500 calls to accumulate enough Mean Squared Error for evaluating parameter search.
 
Different from tuning approach introduced in classroom. dt is calculated for performing proper integral and differentiation. Without applying dt,

The PID control is unstable. Even twiddle algorithm has potential in automated tuning process and may help finding good 
baseline parameters, I ended tuning PID parameters.  A simple PI controller and a Bang Bang speed controller are also implemented()line 147 to 167)
for velocity control.

The final submission was made during aggressively reaching faster speed. So P term is reduced in order to reduce high frequency cross zero behavior like vehicle
wobbling side to sid comparing to base line setup. 

A bang bang controller assumes linear model between steer angle and reverse torque (break) applied to vehicle when it approaching a 
sharp corner. So that the car can leverage reduced orientation correction behavior to increase average speed.

Current parameter:
    
    double _Kp = 0.15;
    double _Ki = 0.25;
    double _Kd = 0.004;

Is tuned for more aggressive driving. If more conservative driving is required, same PID parameters setup with lower nominal throttle  value

/**
 *throttle_value = 0.3
 */
Reading List and references
---

[PID tunning when process dynamics varies](http://techteach.no/presentations/tekna_olje_gass_04/lecture/documents/adaptive_controller.pdf)

[How measure dt in simulator](https://discussions.udacity.com/t/how-to-make-the-pid-output-normalized-to-be-within-1-1/252173/5)

[Application and Analysis of a Robust Trajectory Tracking Controller for Under-Characterized Autonomous Vehicles](http://www.meloneewise.com/wp-content/uploads/2015/08/trajectory_paper.pdf)

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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
