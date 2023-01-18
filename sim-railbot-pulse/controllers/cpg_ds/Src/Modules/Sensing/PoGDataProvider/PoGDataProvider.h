#pragma once

#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Sensing/PoGData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Module/Blackboard.h"

class CPGGeneratorOutput;

class PoGDataProvider {
 public:
  void update(PoGData &pog);

 private:
  REQUIRES_REPRESENTATION(RobotModel);
  REQUIRES_REPRESENTATION(InertialSensorData);
  REQUIRES_REPRESENTATION(CPGGeneratorOutput);
};