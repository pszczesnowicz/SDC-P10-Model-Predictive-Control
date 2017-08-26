This is my submission for the Udacity Self-Driving Car Nanodegree Model Predictive Control (MPC) Project. You can find my C++ source code [here](https://github.com/pszczesnowicz/SDCND-P10-MPC/tree/master/src). The goal of this project was to implement a MPC system that could drive a car around a track in a simulator. A similar task was accomplished by my [Behavioral Cloning](https://github.com/pszczesnowicz/SDCND-P3-BehavioralCloning) and [PID Controller](https://github.com/pszczesnowicz/SDCND-P9-PIDController) projects.

## Model Predictive Control

The MPC system optimizes the car's trajectory based on its current state by calculating trajectory costs for different actuations. The actuation with the lowest cost is then executed. The trajectory is optimized following every actuation using the car's new state.

### State

The state is a vector consisting of the car's position in <a href="https://www.codecogs.com/eqnedit.php?latex=x" target="_blank"><img src="https://latex.codecogs.com/gif.latex?x" title="x" /></a> and <a href="https://www.codecogs.com/eqnedit.php?latex=y" target="_blank"><img src="https://latex.codecogs.com/gif.latex?y" title="y" /></a> coordinates (meters), heading <a href="https://www.codecogs.com/eqnedit.php?latex=\psi" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\psi" title="\psi" /></a> (radians), speed <a href="https://www.codecogs.com/eqnedit.php?latex=v" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v" title="v" /></a> (meters/second), cross track error <a href="https://www.codecogs.com/eqnedit.php?latex=cte" target="_blank"><img src="https://latex.codecogs.com/gif.latex?cte" title="cte" /></a> (meters), and heading error <a href="https://www.codecogs.com/eqnedit.php?latex=e\psi" target="_blank"><img src="https://latex.codecogs.com/gif.latex?e\psi" title="e\psi" /></a> (radians) at time <a href="https://www.codecogs.com/eqnedit.php?latex=t" target="_blank"><img src="https://latex.codecogs.com/gif.latex?t" title="t" /></a>.

<a href="https://www.codecogs.com/eqnedit.php?latex=state=[x_{t},y_{t},\psi_{t},v_{t},cte_{t},e\psi_{t}]" target="_blank"><img src="https://latex.codecogs.com/gif.latex?state=[x_{t},y_{t},\psi_{t},v_{t},cte_{t},e\psi_{t}]" title="state=[x_{t},y_{t},\psi_{t},v_{t},cte_{t},e\psi_{t}]" /></a>

### Actuators

The actuator vector contains the steering <a href="https://www.codecogs.com/eqnedit.php?latex=\delta" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\delta" title="\delta" /></a> (radians) and throttle <a href="https://www.codecogs.com/eqnedit.php?latex=a" target="_blank"><img src="https://latex.codecogs.com/gif.latex?a" title="a" /></a> actuations at time <a href="https://www.codecogs.com/eqnedit.php?latex=t" target="_blank"><img src="https://latex.codecogs.com/gif.latex?t" title="t" /></a>.

<a href="https://www.codecogs.com/eqnedit.php?latex=actuators=[\delta_{t},a_{t}]" target="_blank"><img src="https://latex.codecogs.com/gif.latex?actuators=[\delta_{t},a_{t}]" title="actuators=[\delta_{t},a_{t}]" /></a>

They are bounded by the following limits:

<a href="https://www.codecogs.com/eqnedit.php?latex=\delta&space;\epsilon&space;[-25\pi/180,25\pi/180]" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\delta&space;\epsilon&space;[-25\pi/180,25\pi/180]" title="\delta \epsilon [-25\pi/180,25\pi/180]" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=a&space;\epsilon&space;[-1,&space;1]" target="_blank"><img src="https://latex.codecogs.com/gif.latex?a&space;\epsilon&space;[-1,&space;1]" title="a \epsilon [-1, 1]" /></a>

### Update Equations

The following equations predict the car's future state. They are used to solve the latency problem (explained below) and to constrain the future state and error equations that are sent to the optimizer.

<a href="https://www.codecogs.com/eqnedit.php?latex=x_{t&plus;1}=x_{t}&plus;v_{t}cos(\psi_{t})dt" target="_blank"><img src="https://latex.codecogs.com/gif.latex?x_{t&plus;1}=x_{t}&plus;v_{t}cos(\psi_{t})dt" title="x_{t+1}=x_{t}+v_{t}cos(\psi_{t})dt" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=y_{t&plus;1}=y_{t}&plus;v_{t}sin(\psi_{t})dt" target="_blank"><img src="https://latex.codecogs.com/gif.latex?y_{t&plus;1}=y_{t}&plus;v_{t}sin(\psi_{t})dt" title="y_{t+1}=y_{t}+v_{t}sin(\psi_{t})dt" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=\psi_{t&plus;1}=\psi_{t}&plus;\frac{v_{t}}{L_{f}}\delta_{t}dt" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\psi_{t&plus;1}=\psi_{t}&plus;\frac{v_{t}}{L_{f}}\delta_{t}dt" title="\psi_{t+1}=\psi_{t}+\frac{v_{t}}{L_{f}}\delta_{t}dt" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=v_{t&plus;1}=v_{t}&plus;a_{t}dt" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v_{t&plus;1}=v_{t}&plus;a_{t}dt" title="v_{t+1}=v_{t}+a_{t}dt" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=cte_{t&plus;1}=f(x_{t})-y_{t}&plus;v_{t}sin(e\psi_{t})dt" target="_blank"><img src="https://latex.codecogs.com/gif.latex?cte_{t&plus;1}=f(x_{t})-y_{t}&plus;v_{t}sin(e\psi_{t})dt" title="cte_{t+1}=f(x_{t})-y_{t}+v_{t}sin(e\psi_{t})dt" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=e\psi_{t&plus;1}=\psi_{t}-acrtan(f'(x_{t}))&plus;\frac{v_{t}}{L_{f}}\delta&space;_{t}dt" target="_blank"><img src="https://latex.codecogs.com/gif.latex?e\psi_{t&plus;1}=\psi_{t}-acrtan(f'(x_{t}))&plus;\frac{v_{t}}{L_{f}}\delta&space;_{t}dt" title="e\psi_{t+1}=\psi_{t}-acrtan(f'(x_{t}))+\frac{v_{t}}{L_{f}}\delta _{t}dt" /></a>

### Cost

The cost is a summation of the squares of the <a href="https://www.codecogs.com/eqnedit.php?latex=cte" target="_blank"><img src="https://latex.codecogs.com/gif.latex?cte" title="cte" /></a>, <a href="https://www.codecogs.com/eqnedit.php?latex=e\psi" target="_blank"><img src="https://latex.codecogs.com/gif.latex?e\psi" title="e\psi" /></a>, difference between current and reference velocity, actuations, and difference between sequential actuations for all timesteps <a href="https://www.codecogs.com/eqnedit.php?latex=N" target="_blank"><img src="https://latex.codecogs.com/gif.latex?N" title="N" /></a>.

<a href="https://www.codecogs.com/eqnedit.php?latex=cost=\sum_{t=1}^{N}cte_{t}^2&plus;e\psi_{t}^2&plus;(v_{t}-v_{ref})^2&plus;\delta_{t}^2&plus;a_{t}^2&plus;(\delta_{t&plus;1}-\delta_{t})^2&plus;(a_{t&plus;1}-a_{t})^2" target="_blank"><img src="https://latex.codecogs.com/gif.latex?cost=\sum_{t=1}^{N}cte_{t}^2&plus;e\psi_{t}^2&plus;(v_{t}-v_{ref})^2&plus;\delta_{t}^2&plus;a_{t}^2&plus;(\delta_{t&plus;1}-\delta_{t})^2&plus;(a_{t&plus;1}-a_{t})^2" title="cost=\sum_{t=1}^{N}cte_{t}^2+e\psi_{t}^2+(v_{t}-v_{ref})^2+\delta_{t}^2+a_{t}^2+(\delta_{t+1}-\delta_{t})^2+(a_{t+1}-a_{t})^2" /></a>

### Parameters

The number of timesteps and their duration <a href="https://www.codecogs.com/eqnedit.php?latex=dt" target="_blank"><img src="https://latex.codecogs.com/gif.latex?dt" title="dt" /></a> was set to 10 steps and 0.1 seconds. This predicts the car's state 1 second into the future. During testing, I found this was enough for the car to anticipate a turn while not too far ahead as to cause the car to oscillate while driving on a straight portion of the track.

With less timesteps the car was not able to make sharp turns because the shorter predicted trajectory did not have to be as curved as the reference trajectory to yield a small cost, i.e. the predicted trajectory was more of a straight line tangent to the reference trajectory. With more timesteps the time horizon stretched too far ahead such that the car would turn on a straight portion of the track once a curve has met the time horizon. A shorter timestep duration meant that the number of timesteps had to be increased in order to maintain turning performance and therefore, increase computational time. A longer timestep duration resulted in less precise actuations, i.e. choppier steering.

Experimentation with different cost multipliers lead me to heavily weigh the heading error (x100), steering actuation (x100), and difference between sequential steering actuations (x1000). These three costs most significantly influence the car's steering and in turn can dampen steering oscillation.

Before returning the actuations to the simulator, the throttle is multiplied by a fraction inversely related to the steering angle: a steering angle of 0 degrees yields a multiplier of 1 and 25 degrees yields 1/6. This was the last parameter that I tuned to keep the car from flying off the track on sharp turns while still being able to reach high speeds on the straights.

`throttle = solution.x[a_start] * (M_PI / (fabs(solution.x[delta_start]) * 36.0 + M_PI));`

## Latency

To replicate actuation latency as in a real world self-driving car, the system has a delay of 100 milliseconds before the actuations are sent to the simulator. This causes a problem because the actuations for the car's predicted future state are no longer correct, e.g. a turn command when the car has already made the turn. The solution was to predict the car's state ahead of the latency using the current state and actuations before sending it to the MPC to predict the future trajectory and actuations.

## Results

With only manual weight tuning the car managed to reach a top speed of 95 MPH while staying in control.

[<img src="https://raw.githubusercontent.com/pszczesnowicz/SDCND-P10-MPC/master/readme_images/mpc.jpg" width="800">](https://www.youtube.com/watch?v=SCNWJEqh2Y4&feature=youtu.be "Click to watch")

To improve the MPC system I plan on implementing automatic weight tuning and creating additional waypoints by interpolating. The additional waypoints should yield better polynomial coefficients leading to a smoother reference trajectory and in turn more precise actuations.

## References

[Udacity Self-Driving Car ND](http://www.udacity.com/drive)

[Udacity Self-Driving Car ND - MPC Project](https://github.com/udacity/CarND-MPC-Project)
