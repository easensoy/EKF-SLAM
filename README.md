# EKF-MATLAB
EKF SLAM Implementation in MATLAB

This application is about the implementation of the EKF-SLAM algorithm in a simulated environment and visualises the estimated trajectory and map of landmarks based on sensor measurements. EKF-SLAM is a technique used in robotics to estimate the pose of a robot and create a map of its environment using sensor measurements. 

The code generates control inputs (velocity and yaw rate) and simulates observations of landmarks based on the robot's true pose. It also adds process and observation noise to the inputs and observations, respectively.

![EKF SLAM Result](https://github.com/easensoy/EKF-SLAM/assets/76905667/089e456c-4cdb-4597-b8a3-fca72dcf2b29)

## EKF SLAM:

a. Prediction: It predicts the robot's pose using the motion model and updates the covariance matrix.

b. Update: For each observed landmark, it calculates the innovation (difference between the predicted and observed measurements) and updates the estimate using the Kalman gain. If a new landmark is observed, it is added to the state vector and covariance matrix.

c. Correction: It corrects the estimated pose by adjusting the robot's orientation.

Then, it stores the simulation results (true pose, dead reckoning estimate, EKF SLAM estimate, observations, etc.) for analysis and visualization.

Furthermore, it visualises the simulation results by plotting the true pose, dead reckoning estimate, EKF SLAM estimate, observed landmarks, and true landmarks. It also displays the observed lines between the robot and landmarks.

Basically, it demonstrates the basic implementation of EKF SLAM for estimating the robot's pose and landmark positions in a simulated environment.
