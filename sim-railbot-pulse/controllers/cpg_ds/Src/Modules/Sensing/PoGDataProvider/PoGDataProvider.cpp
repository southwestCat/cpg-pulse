#include "PoGDataProvider.h"

#include "Modules/Motion/config.h"
#include "Representations/PSO/CPGGeneratorOutput.h"

void PoGDataProvider::update(PoGData &pog) {
  //-> Update PoG
  Vector3f com;
  float px = theRobotModel->centerOfMass.x();
  float py = theRobotModel->centerOfMass.y();
  float pz = theRobotModel->centerOfMass.z();
  com << px, py, pz;
  Pose3f soleL = theRobotModel->soleLeft;
  Pose3f soleR = theRobotModel->soleRight;
  Vector3f wOrigin = (soleL.translation + soleR.translation) / 2.f;
  Vector3f cop = -wOrigin + com;
  float ax = pog.ax;
  float bx = pog.bx;
  float ay = pog.ay;
  float by = pog.by;
  pog.PoG.x() = ax * cop.x() + bx;
  pog.PoG.y() = ay * cop.y() + by;

  //-> Update COV Init
  const float initCOVRatio = pog.InitCOVRatio;
  pog.COV << initCOVRatio, initCOVRatio;
  //-> Update COVX
  const float r = pog.ThreshCOV;
  const float h = MotionConfig::comHeihgt;
  const float AccMax = Constants::g_1000 / h * r;  // 2.985527
  //-> Measured AccX
  float accX = theInertialSensorData->acc.x();
  float ratio = abs(accX) / AccMax;
  if (ratio < initCOVRatio) ratio = initCOVRatio;
  if (ratio > 1.f) ratio = 1.f;
  pog.COV.x() = ratio;

  float accY = theInertialSensorData->acc.y();
  float ratioy = abs(accY) / AccMax;
  if (ratioy < initCOVRatio) ratioy = initCOVRatio;
  if (ratioy > 1.0) ratioy = 1.0;
  pog.COV.y() = ratioy;

//   if (theCPGGeneratorOutput->inPul) {
//     pog.COV *= 2.0;
//   }

  //-> Update COVY
  // TODO:
}