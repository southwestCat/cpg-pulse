#pragma once

#include "Tools/Math/Eigen.h"
#include <functional>
#include <vector>
#include <array>
#include <random>
#include <memory>
#include <fstream>

#include "Tools/Module/Blackboard.h"

class Fitness;

class PSO
{
public:
    PSO();
    ~PSO();
    //-> update i-th particle
    void update();
    void initParams();

    //-> costtime
    //-> cumulateUErr: control input cumulative error
    //-> terminal state error, com return to the initial position
    //-> cumulateAErr: cumulative bodyangle and gyro error
    //-> fall: robot falldown
    //-> direction: complianct motion is the same direction with the applied force
    std::function<float(float costtime, float cumulateUErr, float terminalErr, float cumulateAErr, bool fall, bool direction)> func;

    std::vector<float> fitness;
    std::vector<Vector6f> params;

private:
    static const int nParticles = 20;
    float c1;
    float c2;
    float w;

    int particleIndex;
    int iterIndex;
    // bool updateGBest = false;
    bool paramInit = false;

    std::random_device rd;
    std::default_random_engine e;
    std::unique_ptr<std::uniform_real_distribution<float>> u0;
    std::unique_ptr<std::uniform_real_distribution<float>> u1;
    std::unique_ptr<std::normal_distribution<float>> n;

private:
    //->
    std::array<Vector6f, nParticles> X;
    std::array<Vector6f, nParticles> V;
    std::array<Vector6f, nParticles> pbest;
    Vector6f gbest;
    std::array<float, nParticles> pbest_obj;
    float gbest_obj;

    std::ofstream log;

private:
    Vector6f randomX();
    Vector6f randomV();
    void updatePBest(float obj);
    void updateGBest();

public:
    void setX(int index, Vector6f value);
    Vector6f getParams();
    void updateParams();
    void updateObj();

private:
    REQUIRES_REPRESENTATION(Fitness);
};
