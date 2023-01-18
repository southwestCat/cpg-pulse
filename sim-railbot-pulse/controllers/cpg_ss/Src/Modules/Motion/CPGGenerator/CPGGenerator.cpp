#include "CPGGenerator.h"

#include <iostream>

#include "Matsuoka.h"
#include "Modules/Motion/config.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Motion/SupportState.h"
#include "Representations/PSO/CPGGeneratorOutput.h"
#include "Representations/PSO/CoMLogger.h"
#include "Representations/PSO/Fitness.h"
#include "Representations/Sensing/ContactForce.h"
#include "Representations/Sensing/PoGData.h"
#include "Representations/Sensing/ZMPData.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Motion/InverseKinematic.h"

using namespace std;

CPGGenerator::CPGGenerator(float dt) : dt(dt) {
  init();

  // log.open("Logs/CPGGenerator.log");
}

CPGGenerator::~CPGGenerator() {}

void CPGGenerator::init() {
  pulNeuX.reset();
  pulNeuY.reset();

  pulNeuX = make_unique<Matsuoka2Neu>(Matsuoka2Neu::pulsePattern, dt);
  pulNeuY = make_unique<Matsuoka2Neu>(Matsuoka2Neu::pulsePattern, dt);

  pStat = State::standing;
  updateYMax = true;
  contactForceReach = false;

  lasty = 0;

  qx.reset(1.0 / dt);
  qy.reset(1.0 / dt);
}

void CPGGenerator::update(JointRequest &j) {
  return;

  if (!checkContactForce()) {
    theCPGGeneratorOutput->startOuput = false;
    return;
  }

  theCPGGeneratorOutput->startOuput = true;

  switch (pStat) {
    case State::none:
      break;
    case State::standing:
      standing();
      break;
    case State::startPul:
      startPulse();
      break;
    case State::inPul:
      inPulse(j);
      break;
    case State::endPul:
      endPulse();
    case State::recovery:
      recover(j);
    default:
      break;
  }
}

bool CPGGenerator::checkContactForce() {
  if (pStat != State::standing) {
    return true;
  } else {
    theCPGGeneratorOutput->startTime = theFrameInfo->ftime;
    if (theContactForce->norm() > 0.f) {
      contactForceReach = true;
      force = *theContactForce;
      theCPGGeneratorOutput->forceX = force.x();
      // return true;
    } else {
      if (!contactForceReach) contactForceStartTime = theFrameInfo->ftime;
      return false;
    }

    if (contactForceReach) {
      if (theFrameInfo->getTimeSince(contactForceStartTime) > 0.1) {
        return true;
      }
      if (abs(theContactForce->x()) > abs(force.x())) force = *theContactForce;
      return false;
    }
  }
  return true;
}

void CPGGenerator::recover(JointRequest &j) {
  Angle *p;
  Angle *r;
  if (theSupportState->leg == Legs::left) {
    p = &j.angles[Joints::lHipPitch];
    r = &j.angles[Joints::lHipRoll];
  } else {
    p = &j.angles[Joints::rHipPitch];
    r = &j.angles[Joints::rHipRoll];
  }

  float t = theFrameInfo->getTimeSince(tRecoverStart);

  float pitch = pitchStart + (hipPitchInit - pitchStart) * t / RECOVERTIME;
  float roll = rollStart + (hipRollInit - rollStart) * t / RECOVERTIME;

  *p = pitch;
  *r = roll;

  if (t > RECOVERTIME) {
    pStat = State::standing;
  }
}

void CPGGenerator::standing() {
  float lhipP = theJointRequest->angles[Joints::lHipPitch];
  float lhipR = theJointRequest->angles[Joints::lHipRoll];
  float rhipP = theJointRequest->angles[Joints::rHipPitch];
  float rhipR = theJointRequest->angles[Joints::rHipRoll];
  float lankleP = theJointRequest->angles[Joints::lAnklePitch];
  float rankleP = theJointRequest->angles[Joints::rAnklePitch];
  hipPitchInit = theSupportState->leg == Legs::left ? lhipP : rhipP;
  hipRollInit = theSupportState->leg == Legs::left ? lhipR : rhipR;
  anklePitchInit = theSupportState->leg == Legs::left ? rankleP : lankleP;
  pStat = State::startPul;
}

void CPGGenerator::startPulse() {
  Vector6f param =
      (Vector6f() << 3.10412, 0.424391, 9.28835, 9.31659, -2.99902, 1)
          .finished();

  pulNeuX->setParam(param);
  pulNeuX->setInit(0, 0, 0, -0.2);
  ymaxX = pulNeuX->getYAbsMax(MAXCOSTTIME);
  pulNeuX->setInit(1.26932, 1.26933, 1.26932, 1.26933);

  pulNeuY->setParam(param);
  pulNeuY->setInit(0, 0, 0, -0.2);
  ymaxY = pulNeuY->getYAbsMax(MAXCOSTTIME);
  pulNeuY->setInit(1.26932, 1.26933, 1.26932, 1.26933);

  pStat = State::inPul;
}

void CPGGenerator::inPulse(JointRequest &j) {
  //-> Set Feedback Pitch
  float feedX = -theContactForce->x();
  // feedX = 0;
  pulNeuX->setFeed(feedX, -feedX);
  pulNeuX->integrate();
  float yX, u1X, v1X, u2X, v2X;
  tie(yX, u1X, v1X, u2X, v2X) = pulNeuX->yuv();
  yX = yX / ymaxX * Kx;
  // yX = 0;

  //-> Set feedback Y
  float feedY = -theContactForce->y();
  // feedY = 0;
  if (theSupportState->leg == Legs::left) {
    if (feedY > 0) {
      feedY = 0;
    }
  } else if (theSupportState->leg == Legs::right) {
    if (feedY < 0) {
      feedY = 0;
    }
  }
  pulNeuY->setFeed(feedY, -feedY);
  pulNeuY->integrate();
  float yY, u1Y, v1Y, u2Y, v2Y;
  tie(yY, u1Y, v1Y, u2Y, v2Y) = pulNeuY->yuv();
  yY = yY / ymaxY * Ky;
  // yY = 0;

  //-> Terminate
  if (isTerminatedX(yX) && isTerminatedY(yY)) {
    pStat = State::endPul;
  }

  float pitch = hipPitchInit - yX;
  float roll = hipRollInit + yY;
  float ankleP = anklePitchInit - yX;
  if (ankleP > anklePitchInit) ankleP = anklePitchInit;
  makeJoints(j, pitch, roll, ankleP);
}

void CPGGenerator::endPulse() {
  qx.reset(1.0 / dt);
  qy.reset(1.0 / dt);

  tRecoverStart = theFrameInfo->ftime;
  float lhipP = theJointRequest->angles[Joints::lHipPitch];
  float lhipR = theJointRequest->angles[Joints::lHipRoll];
  float rhipP = theJointRequest->angles[Joints::rHipPitch];
  float rhipR = theJointRequest->angles[Joints::rHipRoll];
  pitchStart = theSupportState->leg == Legs::left ? lhipP : rhipP;
  rollStart = theSupportState->leg == Legs::left ? lhipR : rhipR;
  pStat = State::recovery;
  theCPGGeneratorOutput->finish = true;
  theCPGGeneratorOutput->finishTime = theFrameInfo->ftime;
  theCPGGeneratorOutput->terminal = terminalError;
}

void CPGGenerator::reset() { init(); }

bool CPGGenerator::isTerminatedX(float y) {
  bool r = false;
  qx.insert(y);
  if (qx.full()) {
    if (qx.absmax() < tValue || qx.deltaMaxMin() < tdValue) r = true;
  }

  return r;
}

bool CPGGenerator::isTerminatedY(float y) {
  bool r = false;
  qy.insert(y);
  if (qy.full()) {
    if (qy.absmax() < tValue || qy.deltaMaxMin() < tdValue) r = true;
  }
  return r;
}

void CPGGenerator::makeJoints(JointRequest &j, float pitch, float roll, float ankleP) {
  Angle *hp;
  Angle *hr;
  Angle *ap;
  if (theSupportState->leg == Legs::left) {
    hp = &j.angles[Joints::lHipPitch];
    hr = &j.angles[Joints::lHipRoll];
    ap = &j.angles[Joints::rAnklePitch];
  } else {
    hp = &j.angles[Joints::rHipPitch];
    hr = &j.angles[Joints::rHipRoll];
    ap = &j.angles[Joints::lAnklePitch];
  }
  *hp = pitch;
  *hr = roll;
  *ap = ankleP;
}
