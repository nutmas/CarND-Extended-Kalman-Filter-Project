# Extended Kalman Filter(EKF)
## Object tracking with sensor fusion

---

[//]: # [Image References]

[image1]: ./support/SimulatorStartup.png "Simulator Startup Window"


## Overview
Implementation of an extended Kalman filter in C++, which tracks a vehicles position and velocity by fusing simulated Radar and Lidar measurement data. The simulated data is provided over WebSocket from the Udacity Term 2 Simulator. The code fuses this recieved data and predicts the path of the vehicle using an Extended Kalman Filter. The prediction is passed back to the simulator and displayed with the sensor measurments.

---


## Installation steps

To run this code the following downloads are required:

1. Make a project directory `mkdir project_EKF && cd project_EKF`
2. Term 2 Simulator: 
      * Download from here: [Udacity Simulator](https://github.com/udacity/self-driving-car-sim/releases).
      * Place the extracted folder into the project directory. 
      * Within the folder run simulator `term2_sim`; if sucessful the following window should appear:
      ![alt text][image1]{: width="500px"}
3. uWebSocketIO:
      * Install from this repository: [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. 
4. Clone this repository into the project_EKF directory. `git clone https://github.com/nutmas/CarND-Extended-Kalman-Filter-Project.git`

---

## Other Important Dependencies

* cmake >= 3.5
* make >= 4.1 
* gcc/g++ >= 5.4

---

## Build the code

1. 
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 

---

## Usage


1. From terminal window; change to build folder of project folder `cd /build/`
2. Run EKF: `./ExtendedKF `
3. Message appears ` socket` the EKF is not running
4. Run the simulator

From Simulator window select data set 1 or 2. The Animation will start and the vehicle can be seen moving around the screen.
* Red triangles are Lidar measurements
* Blue triangles are Radar measurements
* Green triangles are EKF predictions

---

## Results

From the Simulator winodw the RMSE (Root Mean Square) can be seen for to evaluate the performance of the EKF for the selected dataset.

---
