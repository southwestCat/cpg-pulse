#include "RobotModelProvider.h"

void RobotModelProvider::update(RobotModel &r)
{
    r.setJointData(*theJointSensorData, *theRobotDimensions, *theMassCalibration);
}
