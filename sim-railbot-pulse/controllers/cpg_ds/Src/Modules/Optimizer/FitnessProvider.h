#pragma once

#include "Tools/Module/Blackboard.h"
#include "Tools/Module/Queue.h"

class Fitness;
class FallDownState;
class FrameInfo;
class CPGGeneratorOutput;
class InertialSensorData;

class FitnessProvider
{
public:
    FitnessProvider();
    void update(Fitness &f);
    void reset();
    void setStartTime(float t) 
    {
        if (updatedStartTime)
            return; 
        startTime = t; 
        updatedStartTime = true;
    }

private:
    float dt = 0.01;
    bool updatedStartTime = false;
    float startTime = 0.f;
    Queue q; //-> Check control direction.
    float cumulatedControlError = 0.f;
    float cumulatedBodyAngleError = 0.f;
    float cumulatedGyroError = 0.f;
    std::vector<float> cControl;
    std::vector<float> cBodyAngle;
    std::vector<float> cGyro;

private:
    void resetCumulatedError();

private:
    REQUIRES_REPRESENTATION(FallDownState);
    REQUIRES_REPRESENTATION(FrameInfo);
    REQUIRES_REPRESENTATION(CPGGeneratorOutput);
    REQUIRES_REPRESENTATION(InertialSensorData);
    USES_REPRESENTATION(Fitness);
};