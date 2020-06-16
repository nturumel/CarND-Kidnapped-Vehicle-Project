# Overview
This repository contains all the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.

#### Submission
All you will need to submit is your `src` directory. You should probably do a `git pull` before submitting to verify that your project passes the most up-to-date version of the grading code (there are some parameters in `src/main.cpp` which govern the requirements on accuracy and run time).

## Background
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

# Directory Structure
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```



# Algorithm

Particle filters or Sequential Monte Carlo (SMC) methods are a set of [Monte Carlo](https://en.wikipedia.org/wiki/Monte_Carlo_method) algorithms used to solve [filtering problems](https://en.wikipedia.org/wiki/Filtering_problem_(stochastic_processes)) arising in [signal processing](https://en.wikipedia.org/wiki/Signal_processing) and [Bayesian statistical inference](https://en.wikipedia.org/wiki/Bayesian_inference). The [filtering problem](https://en.wikipedia.org/wiki/Filtering_problem_(stochastic_processes)) consists of estimating the internal states in [dynamical systems](https://en.wikipedia.org/wiki/Dynamical_systems) when partial observations are made, and random perturbations are present in the sensors as well as in the dynamical system.

**Initialisation:** Initializes particle filter by initializing particles to Gaussian distribution around first position and all the weights to 1.

**Prediction:** Predicts the state for the next time step. Delta_t Time between time step t and t+1 in measurements [s].  Array of dimension 3 [standard deviation of x [m], standard deviation of y [m], standard deviation of yaw [rad]], velocity Velocity of car from t to t+1 [m/s], yaw_rate Yaw rate of car from t to t+1 [rad/s].

**UpdateWeights:** Updates the weights for each particle based on the likelihood of the observed measurements. sensor_range Range [m] of sensor. std_landmark[] Array of dimension 2 [Landmark measurement uncertainty [x [m], y [m]]], observations Vector of landmark observations,  map Map class containing map landmarks.

**Resample:** Set a particles list of associations, along with the associations' calculated world x,y coordinates.

