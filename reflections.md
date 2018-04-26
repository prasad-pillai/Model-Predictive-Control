# Reflections

## Model

**State**
State in Model Predictive Control is composed of 6 factors, namely x-position, y-position,
psi-orientation, v-velocity magnitude, cte-cross track error, epsi-orientation error.

**Actuators**
There are two actuators, δ - steering angle and a- throttle. Throttle is a single value
to represent accelation(+ve values) and breaking (-ve values).

**Update equations**
Kinematic vehicle model is used here. The equations for state update for this model 
are as follows:

[equations](images/equations.png)
    
## Timestep Length and Elapsed Duration (N & dt)

**Timestep length (N)**
This is the number of positions in the future the model predicts. As this number increases
more steps in future is predicted. It is observed during manaul trial and error that
as this value is increased the polyline needs to be fitted to a longer line and is
quite error prone and also the polyline predicted is very different every step giving rise 
to oscillatory and jerky motion. If this value is very less then a small polyline is fitted
and is not able to comprehent for large turns. Initially i tried 50 for this value and the 
result was quite jerky. Later i lowered this value to 20 then 15 and finally fixed to 10 
as it gave me a neet motion.

**Elapsed Duration Between Timesteps(dt)**
This is the time gap between each step. I found that if the time gap is small (less than 0.1) the
path is changing suddenly and if it is large (0.2) the path is quite away from the waypoints. Value
between 0.12 and 0.16 was found to be optimal. I choose 0.12

## Polynomial Fitting and MPC Preprocessing

The x,y position received from the simulator is in map space thus they where converted to car's 
coordinate space (main.cpp line 112). Now a 3rd order polynomial is fitted to this positions and CTE and Orientation errors
where found.

## Model Predictive Control with Latency

The simulator is having a latency of 100ms. ie, the control commands gets executed with a delay of 100 ms.
To account for latency the update equations where modified to include this delay (main.cpp line 133).


