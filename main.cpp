#include <iostream>
#include <vector>
#include "biquad_filter.h"
#include "fir_filter.h"
#include "logger.h"
#include "trajectory_generator.h"
#include <thread>

int main()
{
    using clock = std::chrono::steady_clock;
    auto begin = clock::now();
    std::vector<float> kinematicConstarints = {120, 61, 23, 45};
    TrajectoryGenerator trajectoryGenerator(kinematicConstarints);
    trajectoryGenerator.generateTrajectory(0.001);
    auto end = clock::now();

    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms" << std::endl;

    std::vector<float> trajectoryPoints = trajectoryGenerator.getTrajectoryPoints();
    std::vector<float> trajectoryVelocities = trajectoryGenerator.getTrajectoryVelocities();

    Logger logger;
    logger.Log(trajectoryVelocities, "output.txt");

    return 0;
}
