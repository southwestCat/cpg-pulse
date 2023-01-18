#include "FitnessProvider.h"
#include "Representations/PSO/Fitness.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/PSO/CPGGeneratorOutput.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include <iostream>

using namespace std;

#define DEG(x) (x / 3.1415926535897932384626433832795 * 180.0)

FitnessProvider::FitnessProvider()
{
    dt = Blackboard::getInstance().timeStamp_;
    q.reset(1 / dt);
}

void FitnessProvider::update(Fitness &f)
{
    f.terminate = false;

    if (theFallDownState->fall)
    {
        f.terminate = true;
        f.fitness = -1.f;
    }
    if (theFrameInfo->getTimeSince(startTime) > f.MAXCOSTTIME)
    {
        f.fitness = -1.f;
        f.terminate = true;
    }

    //-> Calculate after CPGGenerator was started.
    if (!theCPGGeneratorOutput->startOuput)
        return;

    //-> Cumulate Error
    float control = theCPGGeneratorOutput->control;
    q.insert(control);
    if (q.full() && q.size() > 0)
    {
        float forceX = theCPGGeneratorOutput->forceX;
        if (q.sign() * forceX < 0)
        {
            f.terminate = true;
            f.fitness = -1.f;
        }
        q.reset(0);
    }

    cumulatedControlError += abs(control) / 10;

    float angleY = DEG(theInertialSensorData->angle.y());
    float gyroY = DEG(theInertialSensorData->gyro.y());
    cumulatedBodyAngleError += abs(angleY);
    cumulatedGyroError += abs(gyroY) / 10;

    //-> Calculate Fitness
    if (theCPGGeneratorOutput->finish)
    {
        float startT = theCPGGeneratorOutput->startTime;
        float finishT = theCPGGeneratorOutput->finishTime;
        float costtime = finishT - startT;
        const float maxcosttime = theFitness->MAXCOSTTIME - startT;
        float terminalError = abs(control);
        if (terminalError >= 80.f)
            terminalError = 80.f - 0.01;
        if (terminalError < 1e-4)
            terminalError = 1e-4;

        cumulatedControlError *= dt / costtime;
        cumulatedBodyAngleError *= dt / costtime;
        cumulatedGyroError *= dt / costtime;

        array<float, 5> w = {5.f, 10.f, 3.f, 1.f, 2.f};
        float wsum = 0.f;
        for_each(w.cbegin(), w.cend(), [&wsum](float n){
            wsum += n;
        });
        for_each(w.begin(), w.end(), [&wsum](float& n){
            n /= wsum;
        });
        float f0 = 1 - costtime / maxcosttime;
        float f1 = 1 / exp(terminalError / 10);
        float f2 = 1 - tanh(cumulatedControlError);
        float f3 = 1 - tanh(cumulatedBodyAngleError);
        float f4 = 1 - tanh(cumulatedGyroError);
        float fitness = w[0] * f0 + w[1] * f1 + w[2] * f2 + w[3] * f3 + w[4] * f4;
        f.fitness = fitness;
        f.terminate = true;
    }
}

void FitnessProvider::reset()
{
    updatedStartTime = false;
    q.reset(1 / dt);
    resetCumulatedError();
}

void FitnessProvider::resetCumulatedError()
{
    cumulatedBodyAngleError = 0.f;
    cumulatedControlError = 0.f;
    cumulatedGyroError = 0.f;
}
