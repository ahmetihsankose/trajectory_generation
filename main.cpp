#include <iostream>
#include <vector>
#include "BiquadFilter.h"
#include "FirFilter.h"
#include "Logger.h"
#include "TrajectoryGenerator.h"
#include <thread>

int main()
{
    using clock = std::chrono::steady_clock;
    auto begin = clock::now();
    std::vector<float> kinematicConstarints = {50, 20, 200, 2000};
    TrajectoryGenerator trajectoryGenerator(kinematicConstarints);
    trajectoryGenerator.generateTrajectory(0.001);
    auto end = clock::now();

    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms" << std::endl;

    std::vector<float> trajectoryPositions = trajectoryGenerator.getTrajectoryPositions();
    std::vector<float> trajectoryVelocities = trajectoryGenerator.getTrajectoryVelocities();

    Logger logger;
    logger.Log(trajectoryPositions, "../output.txt");

    return 0;
}
