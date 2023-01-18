#pragma once

#include "Representations/Sensing/PoGData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Tools/Module/Blackboard.h"

class PoGDataProvider
{
public:
    void update(PoGData &pog);
private:
    REQUIRES_REPRESENTATION(RobotModel);
    REQUIRES_REPRESENTATION(InertialSensorData);

    float calibrateY(float y);
};