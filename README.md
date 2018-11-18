# Extended Kalman Filter(EKF)
## Object tracking with sensor fusion

---

[//]: # (Image References)

[image1]: ./support/SimulatorStartup.png "Simulator Startup Window"
[image2]: ./support/EKFRunning.png " Simulator Results"


## Overview
Implementation of an extended Kalman filter in C++, which tracks a vehicles position and velocity by fusing simulated Radar and Lidar measurement data. The simulated data is provided over WebSocket from the Udacity Term 2 Simulator. The code fuses this received data and predicts the path of the vehicle using an Extended Kalman Filter. The prediction is passed back to the simulator and displayed with the sensor measurements.

---

## Installation steps

To run this code the following downloads are required:

1. Make a project directory `mkdir project_udacity && cd project_udacity`
2. Clone this repository into the project_udacity directory. `git clone https://github.com/nutmas/CarND-Extended-Kalman-Filter-Project.git`
3. Setup environment. `cd CarND-Extended-Kalman-Filter-Project\` and launch the script to install uWebSocketIO `./install-mac.sh`. Alternatively for Ubuntu installs launch `./install-ubuntu.sh`. The environment will be installed with these scripts.
4. Download Term 2 Simulator: 
      * Download from here: [Udacity Simulator](https://github.com/udacity/self-driving-car-sim/releases).
      * Place the extracted folder into the project_EKF directory. 
      * Within the folder run simulator `term2_sim`; if successful the following window should appear:
      ![alt text][image1]

---

## Other Important Dependencies

* cmake >= 3.5
* make >= 4.1 
* gcc/g++ >= 5.4

---

## Build the code

1. From the project_udacity folder change to folder created from cloning `cd CarND-Extended-Kalman-Filter-Project`
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 

---

## Usage

After running the script, confirming the operation of the simulator and building the code the EKF program is ready to be run.

1. From terminal window; change to build folder of project `cd ~/project_udacity/CarND-Extended-Kalman-Filter-Project/build/`
2. Run EKF: `./ExtendedKF `
3. Message appears `Listening to port 4567` the EKF is now running and waiting for connection from the simulator
4. Run the simulator `term2_sim`
5. Press 'Select' in the simulator window.
6. In the EKF terminal window `Connected!!!` will now appear.
7. The EKF is now fully operational.

        * Select the dataset 1 or 2
        * Press Start

The Animation will start and the vehicle can be seen moving around the screen.

![alt text][image2]

* Red markers are Lidar measurements
* Blue markers are Radar measurements
* Green markers are EKF predictions

---

## Results

From the Simulator window the RMSE can be seen to evaluate the performance of the EKF for the selected dataset. Running against Dataset 1 this met the accuracy requirements of the project

---

## License

For License information please see the [LICENSE](./LICENSE) file for details

---

