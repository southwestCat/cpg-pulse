#pragma once

#include <Eigen/Eigen>
#include <memory>
#include <array>

#include "Matsuoka.h"
#include "Queue.h"
#include "Gaussian.h"

using Vector6f = Eigen::Matrix<float, 6, 1>;

class Fitness
{
public:
    Fitness();
    float calcFitness(Vector6f param);
    bool terminate() const { return _terminate; }
    void reset();
    void test();
    void info();

private:
    enum class Stat
    {
        none,
        terminate,
        wrongDirection,
        overtime
    };
    const float _dt = 0.01;

    //-> Desired Generated CPG Patter
    const float MAXCOSTTIME = 5.f;
    const float MAXSHOOT = 10.f;
    const float DEFINEDHALFPERIOD = 1.5f;
    const float MAXSLOPE = M_PI / DEFINEDHALFPERIOD * 3;

    bool _terminate = false;
    bool _crossZero = false;

    std::array<float, 4> w;
    std::unique_ptr<Matsuoka2Neu> pulNeu;
    std::unique_ptr<Gaussian> gau;
    std::unique_ptr<Gaussian> gauT;

    Queue qC; // Control Queue
    Queue qD; // Direction Queue

private:
    float _costtime;
    float _maxY;
    float _cumulatedC;
    float _terminalE;
    float _fCostTime;
    float _fTerminalE;
    float _fCumulatedC;
    float _fOverShoot;
    float _fitness;

    float _ly;  // last y value;
};

class FakeFitness
{
public:
    float calcFitness(Vector6f param)
    {
        return param.sum();
    }
};