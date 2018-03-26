# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## MPC Model

Model Predictive Control (MPC) reframes the task of following a 
trajectory as an optimisation problem. It involves simulating
multiple actuator inputs to predict future vehicle states and 
trajectories across multiple timesteps, and selecting the 
states and trajectories with minimum cost at each timestep.

### State

The State vector is composed of 6 values, which are respectively:
* coordinate _x_ of the vehicle
* coordinate _y_ of the vehicle
* current vehicle orientation _psi_
* current vehicle speed _v_
* current cross-track error _cte_
* current orientation error _epsi_
 
 More formally, we have: _s = [x, y, psi, v, cte, epsi]_

 To get to this point we actually performed 2 geometric 
 transformations to express _s_ in _vehicle coordinates_ 
 and not map coordinates.

 #### Transformation To Vehicle Coordinates

 The way points we are given are expressed in map coordinates,
 but to simplify our MPC model and make it relative to our 
 vehicle, and not the map, we need to perform two operations:
 
 1. Translation 
 2. Rotation

 For the translation, we simply subtract the vehicle's 
 map coordinates from the given waypoint:
 
 ```
 double x = xs[i] - px;
 double y = ys[i] - py;
 ```

 The second step is required because in a typical Cartesian 
 coordinate system, the X axis is to our right, while the Y 
 axis is straight ahead. In the vehicle coordinate system 
 however, the X axis is straight ahead (i.e. the vehicle faces 
 in the X direction), while the Y axis is to the left of the 
 vehicle. We therefore need to rotate our waypoint by an angle 
 _theta_, which is the vehicle's orientation _psi_. Here we 
 rotate by _-psi_, as in the simulator, a negative value means a left turn.

 ```
 xs[i] = x * cos(-theta) - sin(-theta) * y;
 ys[i] = x * sin(-theta) + cos(-theta) * y;          
 ```

 Once those operations have been performed, **we have essentially, made our vehicle the center of our coordinate system**.
 Therefore we set our initial vehicle coordinates along with 
 orientation psi to 0.
 ```
  // Now px and py become 0 since they are the center of the system
  px = 0.0;
  py = 0.0;   
  // Same for psi as we have rotated our coordinate system by psi
  psi = 0.0;       
 ```

#### Computation Of Other State Terms

We then attempt to find the the coefficients of the closest 
polynomial of degree 3 that matches our waypoint, and store 
them. From there, we are can compute the cte as well as epsi.

Naturally, the final vector s looks as thus: _s = [0, 0, 0, v, cte, epsi]_

### Timestemps

We have opted for a small prediction horizon by selecting _N = 
25_ and _dt = 0.05_; thus _T = 1.25s_. The reason for those 
values is because our polynomial coefficients and the shape of 
our curve frequently changes, therefore looking up more than a 
few seconds ahead does not help us get a better model: in fact 
it introduces more "noise" to our path and make the car go 
sideways as there is excess information that is not relevant
for our actuator control now.

### Reference Speed

We have set the reference speed to _70 MPH_. While we have been 
able to complete laps at higher speeds, we observed that 70 MPH 
provides the best ride at high speed.
However, from a safety point of view, it would be better to 
drive at lower speeds.

### Model Constraints

The model's input state _s_ has been described above. This state
is actually augmented with the actuator controls _delta_ and 
_a_, which respectively represent the vehicle's steering angle 
and throttle at the next timesteps.

Since MPC rephrases the task of following a trajectory as an 
optimisation problem, we must specify lower/upper bound as well 
as constraints for our variables.

We specify virtually no lower/upper bound for variables in the 
original state vector _s_. However, the steering angle _delta_, 
must in the range [-25, +25] (in degrees).

We also specify different lower and upper bound values for the 
acceleration _a_:
* lower bound is set to -1: we therefore allow a harsh brake 
(in case of an unvoidable safety maneouver)
* Upper bound is set to 0.75: we only allow progressive, gentle 
acceleration and not full blown one (for safety reasons also)

The constraints themselves are all set to 0 as we aim to have
a model that best approximates the "ideal" yellow trajectory 
suggested by the polynomial we computed earlier - therefore we 
aim to satisfy the equations _vars_yellow - vars_mpc = 0_ for 
every one of our variables (_6 * N + 2 * (N - 1)_; 6 input 
state variables, 2 actuator outputs, across all our N 
timestemps).

### Cost Function

The cost function aims to produce a _cost_, which is a a way
to penalise the model to a lesser or greater degree if certain 
set of conditions are not appropriately met. 


#### State Variables In Cost Function

A lot of trial and error went into this area, and we ultimately
decided to incorporate the following elements in our cost 
function:

* Penalise model by _cte^2_: this is because we want to get the 
car to be as much as possible in the middle of the lane (and 
therefore have very low or zero cte if possible)
* Penalise model by _epsi^2_: This is because we expect the 
vehicle's orientation to be as close as possible to our desired 
orientation
* Penalise model by _(speed - desired_speed)^2_: this makes 
sure our vehicle moves and does not stop early if it has zero 
cte and epsi
* Penalise model by _(cte * delta)^2_: we want to compound the 
penalty if either the cte or the steering angle are high. It 
gets even more punitive if both are high

#### Actuators In Cost Function

We also consider actuator values in the cost function and 
proceed with the following:

* Penalise model by _delta^2_: this is because we want to 
minimise turning the steering wheel (e.g. vehicle follows a 
straight path)
* Penalise model by _a^2_: this is because we want to 
minimise acceleration or decelaration and therefore have 0 
throttle (e.g. car is at constant speed in cruise control)
* Penalise model by _(a * delta)^2_: we want to compound the 
penalty if either the throttle and the steering angle are high. It gets even more punitive if both are high

#### Actuator Gradients In Cost Function

Moreover, we also take into account successfive changes in 
actuator values, in order to reduce the risk of "jitter": 
sudden changes in steering angle or throttle values:
* Penalise model by _(cte_t - _cte_t-1)^2__
* Penalise model by _(a_t - _a_t-1)^2__


#### Cost Component Weights

We also realised that some components of our cost function 
should be more important than others. This means we should
increase the penalty the model incurs on some conditions. 
We therefore multiply some of these component costs by a scalar
that denotes its weight. The code below shows our cost function:

```
// First step is to add cte, epsi as well as velocity difference to cost
for (unsigned int t = 0; t < N; t++) {
  cost += 1000 * CppAD::pow(vars[cte_start + t], 2);
  cost += 10000 * CppAD::pow(vars[cte_start + t] * vars[delta_start + t], 2);
  cost += 10000 * CppAD::pow(vars[epsi_start + t], 2);
  cost += 10 * CppAD::pow(vars[v_start + t] - ref_v, 2);
}

// Then we want to minimise the use of actuators for a smoother ride
for (unsigned int t = 0; t < N - 1; t++) {
  cost += 10 * CppAD::pow(vars[delta_start + t], 2);
  cost += 100 * CppAD::pow(vars[a_start + t], 2);
  cost += 100 * CppAD::pow(vars[a_start + t] * vars[delta_start + t], 2);
}

// Finally. we want to minimise sudden changes between successive states
for(unsigned int t = 0; t < N - 2; ++t){
  cost += 10 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);        
  cost += 10 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);        
}
```

We decided to give _much more_ weight to both the compounded _(cte * delta)^2_ and _epsi^2_ cost components. We believe these
two components are critical for a robust MPC.
Moreover, we also attribute a high value to _(cte)^2_, which 
indicates that we strongly believe the car should be as close
as possible to the ideal points in our trajectory (e.g. center 
of lane) .


### Post-MPC Smoothing Of Trajectory

MPC is not a trivial optimization problem. While we managed to
obtain solutions to our MPC problem, we realised that they still
caused the vehicle to oscillate from one end of the lane to the 
other, while still attempting to follow a good trajectory.
While this could be a technically correct drive at low speeds 
(if vehicle remains within bounds), it is not 
particularly pleasant nor safe nor even human-like driving 
pattern, and we expect robots to be better and safer drivers 
than humans.

The gif below shows our car swaying as no post-MPC smoothing
was applied.

![Vehicle Oscillates With No Post-MPC Smoothing](media/mpc_no_post_mpc_smoothing.gif)

We therefore came up with a simple yet effective way to address
this problem: for each predicted timestep _t_ we take the 
moving average of the actuators at _t_ and next _t + n - 1_ (_n 
< N_) steps. In this exercise, setting _n = 7_ (our number of 
timesteps is _N = 25_) produces very good results and elimates 
most of the swaying. Consequently, we must also recompute our 
mpc predicted waypoints and velocity as we updated our actuator 
values. The code below performs the actuator smoothing and 
waypoint recomputation:

```
int steps = 7;
for(unsigned int i = 0; i < N - steps - 1; ++i){
  double sum_steer = 0.0;
  double sum_throttle = 0.0;    
  for(int j = i; j < i + steps; ++j){
    sum_steer += next_steers[j];
    sum_throttle += next_throttles[j];      
  }
  next_steers[i] = sum_steer / steps;
  next_throttles[i] = sum_throttle / steps;

  // Recalculate v
  double v = solution_vector[v_start + i] + next_throttles[i] * dt;
  
  // Now recalculate next points
  next_xs[i] = solution_vector[x_start + i] + v * cos(next_steers[i]) * dt; 
  next_ys[i] = solution_vector[y_start + i] + v * sin(next_steers[i]) * dt; 
}
```

You can see on the gif below how much smoother the trajectory 
(and ride) is with _post-mpc actuator smoothing_:

![Vehicle Adopts Smooth Ride With Post-MPC Smoothing](media/mpc_post_mpc_actuator_smoothing.gif)


### Dealing with Latency

There is a 100ms delay between emitting commands to the 
actuators and their execution. In the code this was addressed
by introducing a sleep of 100ms. However, we can take this into
our model.

To deal with delay without introducing any sleep instructions,
we need to be able to compute the vehicle's state 100ms 
**after** we have calculated its state at _t=0_ by fitting
a polynomial across the waypoints. For instance, incorporating
this delay, and therefore extra motion, enables us to compute
future position of the vehicle. Let _(x_t_, y_t)_ denote the
vehicle's current position, and _(x_t'_, y_t')_ be the actual 
position of the vehicle, by  taking into account the latency 
_l_ (_l = 0.1 seconds_), speed _v_, and vehicle steering angle 
_delta_.

We would calculate those coordinates as:
```
_x_t' = x_t + v * cos(delta) * l)_
_y_t' = y_t + v * sin(delta) * l)_
```

Moreover, we need to also incorporate this delay into the rest
of our state variables. This is shown in the code excerpt below:
```
// Since we incur a delay of 100ms before the actuator runs,
// we need to take this into account. This means our vehicle 
// has actually moved in the last 100 milliseconds.
// Therefore we must recompute its state 100ms later
px +=  v * cos(delta) * latency;
py += v * sin(delta) * latency;
  
// Likewise, we must recompute the rest of the state
cte = cte + v * sin(epsi) * latency;
epsi = epsi + (v / 2.67) * latency;
psi += (v / 2.67) * delta *  latency;
v +=  a * latency;
```

This nicely accounts for the vehicle moving during the 100ms delay between the command and actual execution of our actuator.


### Conclusion

This was a very interesting project that built on the work done
on PID controllers to better understand how vehicle forces and 
state could be modelled and therefore, in a simple example 
devoid of outside forces and in a relatively static environment,
 demonstrated that we can create a model that is able to predict
a good path and actuate the vehicle to follow this trajectory.

The choice of components in our cost function, especially the 
weights attributed to each of them, was particularly tricky
and we had to rely on our intuition and results on the simulator
to select appropriate values.

Moreover, the _post-mpc actuator smoothing_ was introduced to
eliminate vehicle swaying. We believe latency could be dealt 
with appropriately by accounting for it in the vehicle movement
model. Our impression is that in real life scenarios, actual 
measured latency would probably follow a gaussian distribution (e.g. _mean = 100ms, std = 2ms_). 
This can also be accounted for with a more sophisticated model.

This model does not deal with dynamic environment where in 
real-life we would have to take into account other vehicles, 
pedestrians, etc. Nonetheless, it gives a strong foundational
understanding of vehicle control and will help in the design of
autonomous vehicle system that can operate in the real world.

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
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
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

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

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
