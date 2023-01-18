#pragma once

#include "Modules/Motion/config.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/RobotParts/Joints.h"

class FrameInfo;
class PoGData;
class ZMPData;
class RobotDimensions;
class SupportState;

class SingleSupportGenerator {
 public:
  SingleSupportGenerator(float dt, Legs::Leg leg = Legs::left)
      : _dt(dt), _leg(leg) {}
  void update(JointRequest &j);
  bool finished() { return isFinish; }

 private:
  void lean(JointRequest &j, float t);
  void liftFoot(JointRequest &j, float t);
  void liftAnkle(JointRequest &j, float t);

 private:
  REQUIRES_REPRESENTATION(FrameInfo);
  REQUIRES_REPRESENTATION(RobotDimensions);
  REQUIRES_REPRESENTATION(ZMPData);
  REQUIRES_REPRESENTATION(PoGData);

  MODIFIES_REPRESENTATION(SupportState);

 private:
  const float _dt;
  Legs::Leg _leg;
  const float h = MotionConfig::hipHeight;
  const float tLEAN = 1.0;
  const float tLIFT = 1.0;
  const float tANKLE = 0.5;
  bool isFinish = false;
  float ankleP;
  Angle *ap;

  float t = 0;

  Vector3f WPB;
  Vector3f WPLF;
  Vector3f WPRF;
  Vector3f lWPLF;
  Vector3f lWPRF;
};