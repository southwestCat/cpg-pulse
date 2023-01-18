#include "SingleSupportGenerator.h"

#include <iostream>

#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Motion/SupportState.h"
#include "Representations/Sensing/PoGData.h"
#include "Representations/Sensing/ZMPData.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Motion/InverseKinematic.h"
using namespace std;

void SingleSupportGenerator::update(JointRequest &j) {
  theSupportState->leg = _leg;
  if (t < tLEAN) {
    lean(j, t);
  } else if (t < tLEAN + tLIFT) {
    liftFoot(j, t - tLEAN);
  // } else if (t < tLEAN + tLIFT + tANKLE) {
  //   liftAnkle(j, t - tLIFT - tLEAN);
  } else {
    isFinish = true;
  }
  t += _dt;
}

void SingleSupportGenerator::lean(JointRequest &j, float t) {
  const float comBias = 50.0;
  float comShift = _leg == Legs::left ? comBias : -comBias;
  float y = t / tLEAN * comShift;

  //-> WPB
  WPB = Vector3f(0, y, MotionConfig::hipHeight);
  WPLF = Vector3f(0, theRobotDimensions->yHipOffset, 0);
  WPRF = Vector3f(0, -theRobotDimensions->yHipOffset, 0);

  Vector3f BPLF = -WPB + WPLF;
  Vector3f BPRF = -WPB + WPRF;

  Pose3f targetL(BPLF);
  Pose3f targetR(BPRF);

  JointRequest _j;
  bool isPossible = InverseKinematic::calcLegJoints(
      targetL, targetR, Vector2f::Zero(), _j, *theRobotDimensions);
  if (isPossible) {
    for (int i = Joints::firstLegJoint; i <= Joints::rAnkleRoll; i++) {
      j.angles[i] = _j.angles[i];
    }
  }
}

void SingleSupportGenerator::liftFoot(JointRequest &j, float t) {
  float z = t / tLIFT * 20;
  if (_leg == Legs::left)
    WPRF = Vector3f(0, -theRobotDimensions->yHipOffset, z);
  else if (_leg == Legs::right)
    WPLF = Vector3f(0, theRobotDimensions->yHipOffset, z);
  Vector3f BPLF = -WPB + WPLF;
  Vector3f BPRF = -WPB + WPRF;

  Pose3f targetL(BPLF);
  Pose3f targetR(BPRF);

  JointRequest _j;
  bool isPossible = InverseKinematic::calcLegJoints(
      targetL, targetR, Vector2f::Zero(), _j, *theRobotDimensions);
  if (isPossible) {
    for (int i = Joints::firstLegJoint; i <= Joints::rAnkleRoll; i++) {
      j.angles[i] = _j.angles[i];
    }
  }
  if (_leg == Legs::left) {
    ankleP = j.angles[Joints::rAnklePitch];
    ap = &j.angles[Joints::rAnklePitch];
  } else {
    ankleP = j.angles[Joints::lAnklePitch];
    ap = &j.angles[Joints::lAnklePitch];
  }
}

void SingleSupportGenerator::liftAnkle(JointRequest &j, float t) {
  float p = t / tANKLE * 10_deg;
  *ap = ankleP - p;
}
