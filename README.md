# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

This project utilizes a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.

## Prerequisites
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

## Development Setup: Eclipse CDT4
This project was modified and built using the Eclipse for C++ Development environment. In order to use the IDE, CMakeLists.txt was moved from the main directory to the `/src` subdirectory. Additionally, CMakeLists.txt was modified to build in Eclipse. These modifications follow the direction provided in the IDE guidelines located [here](https://github.com/itscherylanne/CarND-Extended-Kalman-Filter-Project/blob/master/ide_profiles/Eclipse/README.md).

## Building and Running the kalman filter software
Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project's build directory.

```
mkdir build
cd build
cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../src/
make
./ExtendedKF
```
Launch the Term 2 Simulator, select the EKF/Unscented Kalman Filter Project, and run the simulator.

## Program Input and Outputs
INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]


## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake -G "Eclipse CDT4 - Unix Makefiles" -D && make`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `
