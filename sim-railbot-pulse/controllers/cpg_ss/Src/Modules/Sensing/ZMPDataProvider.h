#pragma once

#include "Representations/Sensing/ZMPData.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Tools/Module/Blackboard.h"

class ZMPDataProvider
{
public:
    void update(ZMPData &zmp);

private:
    REQUIRES_REPRESENTATION(FsrSensorData);
    REQUIRES_REPRESENTATION(RobotDimensions);
};