# EKF-MATLAB
EKF SLAM Implementation in MATLAB

Overall, this code implements the EKF-SLAM algorithm in a simulated environment and visualizes the estimated trajectory and map of landmarks based on sensor measurements. EKF-SLAM is a technique used in robotics to estimate the pose of a robot and create a map of its environment using sensor measurements. 

## The code is depicted below

### Initialization

Various variables and parameters are defined, including simulation time, simulation end time, time step, number of simulation steps, and storage variables for the calculation results.

### State Vector and Covariance Initialization

The state vector represents the estimated pose of the robot and is initialized as [x, y, yaw] (position and orientation).
The true state vector represents the ground truth pose of the robot and is initially set to the estimated pose.
The dead reckoning state vector (xd) is also initialized as the true state.

### Covariance Matrices

The covariance matrix (R) is used for the prediction step and models the uncertainty in the motion model.
The covariance matrix (Q) is used for the observation step and models the uncertainty in the sensor measurements.

### Simulation Parameters

Additional parameters related to the simulation, such as noise standard deviations for process and measurement noise, are defined.

### Landmark Positions

The positions of landmarks in the environment are specified.

### Main Loop

The main loop iterates over the simulation steps.
Control inputs (u) are obtained using the doControl function.
Observations (z) are simulated using the Observation function.

### EKF-SLAM algorithm steps are performed
Prediction: The state vector is updated using the motion model (function f) and the Jacobian matrix of the motion model (function jacobF).
Update: For each observed landmark, the algorithm calculates the innovation, Mahalanobis distance, and updates the state and covariance matrix accordingly.
The simulation results are stored for visualization.

### Animation and Visualization

The Animation function is called to plot the true trajectory, dead reckoning trajectory, estimated trajectory, observed landmarks, and true landmarks.
The DrawGraph function is called to plot the true trajectory, dead reckoning trajectory, estimated trajectory, and true landmarks.
