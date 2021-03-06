# **MPC**

## Implementation

### The Model
We use the kinematic model presented in the lesson. Equations are as follows:

```
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```

Where:
- `x, y` : Car's position.
- `psi` : Car's heading direction.
- `v` : Car's velocity.
- `cte` : Cross-track error.
- `epsi` : Orientation error.

These variables are the state vector, with `Lf`	being the distance between the car center of mass and the front wheels.
In addition we also have the actuators:
- `a` : Acceleration.
- `delta` : Steering angle.

The objective is to minimize the cost function by tweaking `a` and `delta`.
The cost function consist of the following factors:
- `cte` and `epsi`.
- Difference to a reference velocity. This is to avoid a stop due to other errors being 0.
- Euclidean distance from initial state to final point given by the reference trajectory. This is to avoid a stop due to other errors being 0.
- Minimize the use of the actuators, as well as penalizing high steering angles at high velocities.
- Minimize the value gap of consecutive actuations to get smooth actuations.

### Timestep Length and Elapsed Duration (N & dt)

I ended up with `N = 20` and `dt = 0.1` after trying some combinations. Lower `N` values didn't provide enough prediction value to get
good performance. Higher `N` values take into account multiple future possibilities, the problem is it requires more computational power to use.
In a real life scenario it's unlikely that we use high `N` values since it would be impossible to predict other elements such as other cars changing lanes,
accidents, etc.
Low `dt` values let us have more actuations but reduce the time horizon, as well as being more computationally expensive. The opposite applies to high `dt` values.
A balance has to be found such that the time horizon is large enough to predict future events without being too computationally expensive, but not
large enough that calculations further in the time horizon carry high uncertainty.

### Polynomial Fitting and MPC Preprocessing
I transformed the waypoints to the car local coordinate system, thus making calculations easier, since the state vector always has `x=0` and `y=0`.
As suggested during the lesson, I fit a third degree polynomial to the waypoints since it will correctly fit most trajectories found in real life scenarios.

### Model Predictive Control with Latency
The kinematic model was used to calculate the next state after the initial. The new state was then fed to the solver to account for latency.

### The vehicle must successfully drive a lap around the track.
https://youtu.be/aC8y3Pf3s80
