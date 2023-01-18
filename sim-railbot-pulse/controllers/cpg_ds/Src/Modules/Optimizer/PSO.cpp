#include "PSO.h"
#include <ctime>
#include <iostream>
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/PSO/Fitness.h"

using namespace std;

PSO::PSO()
{
    //-> Init params
    c1 = 0.1f;
    c2 = 0.1f;
    w = 0.8f;

    //-> pIndex
    particleIndex = 0;
    iterIndex = 0;

    e.seed(rd());
    u0 = make_unique<uniform_real_distribution<float>>(-1.0, 1.0);
    u1 = make_unique<uniform_real_distribution<float>>(0, 1.0);
    n = make_unique<normal_distribution<float>>(0, 1.0);

    log.open("Logs/PSO.log", std::ios::app | std::ios::out);
}

PSO::~PSO()
{
    log.close();
}

Vector6f PSO::randomX()
{
    Vector6f x;
    for (int i = 0; i < 6; i++)
    {
        // x[i] =  * 10.0;
        //->
        x[i] = (*u0)(e)*10.0;
    }
    return x;
}

Vector6f PSO::randomV()
{
    Vector6f v;
    for (int i = 0; i < 6; i++)
    {
        // x[i] =  * 10.0;
        //->
        v[i] = (*n)(e)*0.1;
    }
    return v;
}

void PSO::initParams()
{
    if (paramInit)
        return;

    for (int i = 0; i < nParticles; i++)
    {
        X[i] = randomX();
        V[i] = randomV();
    }

    X[0] << 0.6f, 0.4f, 1.f, 1.8f, 1.6f, 1.f;

    paramInit = true;
}

Vector6f PSO::getParams()
{
    return X[particleIndex];
}

void PSO::setX(int index, Vector6f value)
{
    X.at(index) = value;
}

void PSO::updateParams()
{
    if (iterIndex == 0)
    {
        return;
    }

    float r1 = (*u1)(e);
    float r2 = (*u1)(e);
    V[particleIndex] = w * V[particleIndex] + c1 * r1 * (pbest[particleIndex] - X[particleIndex]) + c2 * r2 * (gbest - X[particleIndex]);
    X[particleIndex] = X[particleIndex] + V[particleIndex];
}

void PSO::updateObj()
{
    const float obj = theFitness->fitness;
    Vector6f p = getParams();
    printf("param: %f %f %f %f %f %f , fitness: %f\n", p[0], p[1], p[2], p[3], p[4], p[5], obj);
    if (iterIndex == 0) //-> Get Initial Fitness
    {
        pbest[particleIndex] = X[particleIndex];
        pbest_obj[particleIndex] = obj;
    }
    else
    {
        updatePBest(obj);
        updateGBest();
    }
}

void PSO::update()
{
    updateObj();

    particleIndex++;
    if (particleIndex == nParticles)
    {
        iterIndex++;
        particleIndex = 0;
    }

    updateParams();

    // const float obj = theFitness->fitness;

    // if (particleIndex == nParticles)
    // {
    //     iterIndex++;
    //     particleIndex = 0;
    // }

    // if (iterIndex == 0) //-> Get Initial Fitness
    // {
    //     pbest[particleIndex] = X[particleIndex];
    //     pbest_obj[particleIndex] = obj;

    //     if (particleIndex == nParticles - 1)
    //     {
    //         updateGBest();
    //     }
    // }
    // else //-> Update PSO
    // {
    //     updatePBest(obj);
    //     updateGBest();
    // }
}

void PSO::updatePBest(float obj)
{
    if (pbest_obj[particleIndex] < obj)
    {
        pbest_obj[particleIndex] = obj;
        pbest[particleIndex] = X[particleIndex];
    }
}

void PSO::updateGBest()
{
    float max_pbest_obj = pbest_obj[0];
    for (int i = 0; i < nParticles; i++)
    {
        if (pbest_obj[i] > max_pbest_obj)
        {
            max_pbest_obj = pbest_obj[i];
            gbest = pbest[i];
            gbest_obj = max_pbest_obj;
        }
    }
}
