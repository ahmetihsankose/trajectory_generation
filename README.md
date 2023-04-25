# Trajectory Generation with FIR Filters

This C++ project demonstrates trajectory generation for single-axis systems, using Finite Impulse Response (FIR) filters under specific kinematic constraints. The aim is to provide smooth and efficient motion trajectories while considering various constraints such as maximum velocity, acceleration, and jerk.

The project is inspired by the following references:

1. Luigi Biagiotti (2023). Trajectory Toolbox (https://www.mathworks.com/matlabcentral/fileexchange/71406-trajectory-toolbox), MATLAB Central File Exchange. Retrieved April 1, 2023.
2. Biagiotti, Luigi, and Claudio Melchiorri. "Trajectory Generation via FIR Filters: A Procedure for Time-Optimization under Kinematic and Frequency Constraints." Control Engineering Practice, vol. 87, Elsevier BV, June 2019, pp. 43–58, doi:10.1016/j.conengprac.2019.03.017.

## Features

- Generates trajectories considering kinematic constraints
- Applies FIR filters for smooth trajectory generation
- Implements Biquad filters for efficient filtering
- Optimized for performance and parallel processing

## Dependencies

- A C++ compiler with support for C++11 or later

## Example Usage

Below is an example of how to use the `TrajectoryGenerator` class to generate a trajectory under given kinematic constraints and display the resulting positions, velocities, accelerations, and jerk values.

```cpp
#include <iostream>
#include <vector>
#include "TrajectoryGenerator.h"

int main()
{
    // Define kinematic constraints
    std::vector<float> kinematicConstraints = {50, 20, 200, 2000}; // Max Distance, Max Velocity, Max Acceleration, Max Jerk

    // Create a trajectory generator instance
    TrajectoryGenerator trajectoryGenerator(kinematicConstraints);

    // Generate the trajectory with a sampling time of 0.001 seconds
    float samplingTime = 0.001;
    trajectoryGenerator.generateTrajectory(samplingTime);

    // Retrieve the generated trajectory data
    std::vector<float> positions = trajectoryGenerator.getTrajectoryPositions();
    std::vector<float> velocities = trajectoryGenerator.getTrajectoryVelocities();
    std::vector<float> accelerations = trajectoryGenerator.getTrajectoryAccelerations();
    std::vector<float> jerk = trajectoryGenerator.getTrajectoryJerk();

    // Display the generated trajectory data
    std::cout << "Generated trajectory: " << std::endl;
    for (size_t i = 0; i < positions.size(); i++)
    {
        std::cout << "Position: " << positions[i]
                  << ", Velocity: " << velocities[i]
                  << ", Acceleration: " << accelerations[i]
                  << ", Jerk: " << jerk[i] << std::endl;
    }

    return 0;
}
