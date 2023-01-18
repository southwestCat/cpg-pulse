#pragma once

#include <fstream>
#include <memory>

#include "Tools/Math/Eigen.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Module/Queue.h"

class ZMPData;
class PoGData;
class FrameInfo;
class JointRequest;
class Matsuoka2Neu;
class CPGGeneratorOutput;
class RobotDimensions;
class ContactForce;
class Fitness;
class SupportState;

class CPGGenerator {
 public:
  CPGGenerator(float dt);
  ~CPGGenerator();
  void update(JointRequest &j);
  void reset();

 private:
  REQUIRES_REPRESENTATION(ZMPData);
  REQUIRES_REPRESENTATION(PoGData);
  REQUIRES_REPRESENTATION(FrameInfo);
  REQUIRES_REPRESENTATION(RobotDimensions);
  REQUIRES_REPRESENTATION(ContactForce);
  REQUIRES_REPRESENTATION(SupportState);

  USES_REPRESENTATION(Fitness);
  USES_REPRESENTATION(JointRequest);

  MODIFIES_REPRESENTATION(CPGGeneratorOutput);

 private:
  enum class State { none, recovery, standing, startPul, inPul, endPul, reset };

  State pStat;

  float dt;
  float lasty;
  float terminalError;
  float contactForceStartTime;
  bool contactForceReach = false;
  Vector2f force;
  std::unique_ptr<Matsuoka2Neu> pulNeuX;  //-> Pitch
  std::unique_ptr<Matsuoka2Neu> pulNeuY;  //-> Roll

  bool updateYMax = true;

  std::ofstream log;

  Queue qx;
  Queue qy;

  const float MAXCOSTTIME = 5.0;
  const float RECOVERTIME = 1.0;
  const float Kx = 5_deg;
  const float Ky = 5_deg;
  const float tValue = 1;
  const float tdValue = 0.01;
  float ymaxX;
  float ymaxY;

  float hipPitchInit;
  float hipRollInit;
  float anklePitchInit;
  float pitchStart;
  float rollStart;
  float tRecoverStart;

 private:
  void standing();
  void recover(JointRequest &j);
  void startPulse();
  void inPulse(JointRequest &j);
  void endPulse();
  void init();
  bool checkContactForce();
  bool isTerminatedX(float y);
  bool isTerminatedY(float y);
  void makeJoints(JointRequest &j, float pitch, float roll, float ankleP);
};
