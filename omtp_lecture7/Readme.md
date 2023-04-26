# simple_dmp

This is an (almost) minimal implementation of DMPs in Cartesian space (both position and quaternion) and in Joint space, with an optional (default disabled) roto-dilatation term.


## Dependencies
This project depends on:
* numpy
* matplotlib
* pandas
* dmp
* roboticstoolbox
* spatialmath-python 
* mujoco
* glfw

To install these, run <code>pip3 install --user numpy numpy-quaternion matplotlib pandas</code>.	

## Running
To the run the project, navigate to the root directory and call <code>python3 main.py</code>. This will first train and plot a DMP in joint space and then do the same in Cartesian space for the provided demonstration file, demonstration.csv.


## Dmp Library

### Canonical System
This Python module provides a CanonicalSystem class for simulating a dynamical system with a single scalar variable.

-------------------

### DMP_Joint

This Python module provides a `JointDMP` class for simulating Dynamic Movement Primitives (DMPs) for robot joint control. DMPs are a trajectory generation method that can adapt to changing environments and are used for learning and generating complex movements.

In the `step()` function of the `JointDMP` class, the Dynamic Movement Primitive (DMP) system acceleration is calculated. There are two cases based on the value of the `FX` argument:

1. If `FX` is `True` (Case 1: fp(x) is enabled), the DMP system acceleration `self.ddp` is calculated using the following formula:

```
self.ddp = (self.alpha * (self.beta * (self.gp - self.p) - tau * self.dp) + fp(x)) / tau**2
```

In this case, the forcing term `fp(x)` is used in the acceleration calculation.

2. If `FX` is `False` (Case 2: fp(x) is disabled), the DMP system acceleration `self.ddp` is calculated using the following formula:

```
self.ddp = (self.alpha * (self.beta * (self.gp - self.p) - tau * self.dp)) / tau**2
```

In this case, the forcing term `fp(x)` is not used in the acceleration calculation, leading to a simple point-to-point trajectory.



### Exercise 1

This script demonstrates how to use the `JointDMP` class from the `dmp_joint` module to generate and adapt robot joint trajectories. It reads a trajectory from a CSV file, trains a DMP based on this trajectory, simulates the DMP, and visualizes the results.


- Define the time step `dt` and calculate the total time `tau` and the time steps `ts`:

```python
dt = 1 / 100
tau = len(demo) * dt
ts = np.arange(0, tau, dt)
```

- Initialize the DMP object with the desired parameters:

```python
dmp_q = dmp_joint.JointDMP(NDOF=7, n_bfs=100, alpha=48, beta=12, cs_alpha=cs_alpha)
```

- Simulate the DMP and obtain the joint positions, velocities, and accelerations:

- Modify the duration and goal position of the DMP to compare the original and adapted trajectories:


### Exercise 2 

This Python script demonstrates the use of Dynamic Movement Primitives (DMPs) in both joint and Cartesian space for UR 5 Robot. It reads a demonstration file (demonstration.csv) containing joint and Cartesian position, as well as orientation data, and trains and generates trajectories using DMPs. The resulting trajectories are then plotted for comparison.

### Exercise 3 
This script will open a mujoco simulation with a franka robot.The robot joints will be controlled to move like a JTrajectory.csv file using DMP.

    Exercise3_demo_DMP_franka.py 


Outputs of the exercises are in the omtp_lecture7/Exercise_Outputs . 
