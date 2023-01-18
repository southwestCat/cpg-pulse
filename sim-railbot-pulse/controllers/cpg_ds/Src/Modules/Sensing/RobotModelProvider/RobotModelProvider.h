#pragma once

#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Module/Blackboard.h"

class RobotModelProvider
{
public:
    void update(RobotModel &robotModel);
private:
    REQUIRES_REPRESENTATION(JointSensorData);
    REQUIRES_REPRESENTATION(RobotDimensions);
    REQUIRES_REPRESENTATION(MassCalibration);
};