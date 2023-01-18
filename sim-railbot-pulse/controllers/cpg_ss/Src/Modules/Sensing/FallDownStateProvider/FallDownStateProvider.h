#pragma once

#include "Representations/Sensing/FallDownState.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Sensing/GroundContactState.h"

#include "Tools/Module/Blackboard.h"

class FallDownStateProvider
{
public:
    void update(FallDownState &f);

private:
    REQUIRES_REPRESENTATION(InertialSensorData);
    REQUIRES_REPRESENTATION(GroundContactState);
};