#include "Nao.h"

#include <webots/PositionSensor.hpp>

#include "Modules/Motion/config.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/PSO/CPGGeneratorOutput.h"
#include "Representations/PSO/Fitness.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Motion/ForwardKinematic.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/RobotParts/FsrSensors.h"

using namespace webots;
using namespace std;

Nao::Nao() {
  // Get Time step
  timeStep = getBasicTimeStep();
  dt = timeStep / 1000.0;
  timeStamp_ = (float)dt;
  Blackboard::getInstance().timeStamp_ = timeStamp_;

  //-> Pattern Generator
  // generator = new SinGenerator(dt);
  // generator = std::make_unique<SinGenerator>(dt);
  generator = make_unique<CPGGenerator>(dt);
  ssgenerator = make_unique<SingleSupportGenerator>(dt, Legs::left);

  //-> Initial Webots Devices
  initDevice();

  log.open("Logs/ss.txt");
}

Nao::~Nao() { log.close(); }

void Nao::setInitialTransAndRot() {
  Node *robotNode = getFromDef("NAO");
  Field *transField = robotNode->getField("translation");
  Field *rotField = robotNode->getField("rotation");
  transField->setSFVec3f(INITIAL_TRANS);
  rotField->setSFRotation(INITIAL_ROT);
}

void Nao::initDevice() {
  //-> Display
  display = getDisplay("display");
  // Node* PLOTNode = getFromDef("PLOT");
  plot.reset();
  plot = make_unique<DebugDraw>(display);
  // plot = new DebugDraw(display);

  // Camera
  cameraTop = getCamera("CameraTop");
  cameraTop->enable(timeStep);
  cameraBottom = getCamera("CameraBottom");
  cameraBottom->enable(timeStep);
  // Accelerometer
  accelerometer = getAccelerometer("accelerometer");
  accelerometer->enable(timeStep);
  // Gyro
  gyro = getGyro("gyro");
  gyro->enable(timeStep);
  // GPS
  gps = getGPS("gps");
  gps->enable(timeStep);
  // Inertial unit
  inertialUnit = getInertialUnit("inertial unit");
  inertialUnit->enable(timeStep);
  // Ultrasound sensors
  distanceSensor[0] = getDistanceSensor("Sonar/Left");
  distanceSensor[1] = getDistanceSensor("Sonar/Right");
  distanceSensor[0]->enable(timeStep);
  distanceSensor[1]->enable(timeStep);
  // Emitter
  emitter = getEmitter("emitter");
  // Receiver
  receiver = getReceiver("receiver");
  receiver->enable(timeStep);
  // Foot FSR sensor
  fsr[0] = getTouchSensor("LFsr");
  fsr[0]->enable(timeStep);
  fsr[1] = getTouchSensor("RFsr");
  fsr[1]->enable(timeStep);
  // Foot Bumper
  lFootLBumper = getTouchSensor("LFoot/Bumper/Left");
  lFootRBumper = getTouchSensor("LFoot/Bumper/Right");
  rFootLBumper = getTouchSensor("RFoot/Bumper/Left");
  rFootRBumprt = getTouchSensor("RFoot/Bumper/Right");
  lFootLBumper->enable(timeStep);
  lFootRBumper->enable(timeStep);
  rFootLBumper->enable(timeStep);
  rFootRBumprt->enable(timeStep);
  // There are 7 controlable LED groups in Webots
  leds[0] = getLED("ChestBoard/Led");
  leds[1] = getLED("RFoot/Led");
  leds[2] = getLED("LFoot/Led");
  leds[3] = getLED("Face/Led/Right");
  leds[4] = getLED("Face/Led/Left");
  leds[5] = getLED("Ears/Led/Right");
  leds[6] = getLED("Ears/Led/Left");
  // Get phalanx motor.
  // the real Nao has only 2 motors for LHand/RHand
  // but in Webots we must implement LHand/RHand with 2x8 motors
  for (int i = 0; i < PHALANX_MAX; i++) {
    std::string name;
    name = "LPhalanx" + std::to_string(i + 1);
    lPhalanx[i] = getMotor(name);
    name = "RPhalanx" + std::to_string(i + 1);
    rPhalanx[i] = getMotor(name);
    maxPhalanxMotorPosition[i] = rPhalanx[i]->getMaxPosition();
    minPhalanxMotorPosition[i] = rPhalanx[i]->getMaxPosition();
  }
  // Joints Motors
  std::string motorNames[Joints::numOfJoints] = {
      "HeadYaw",        "HeadPitch",     "LShoulderPitch", "LShoulderRoll",
      "LElbowYaw",      "LElbowRoll",    "LWristYaw",
      "LHand",  // LHand
      "RShoulderPitch", "RShoulderRoll", "RElbowYaw",      "RElbowRoll",
      "RWristYaw",
      "RHand",  // RHand
      "LHipYawPitch",   "LHipRoll",      "LHipPitch",      "LKneePitch",
      "LAnklePitch",    "LAnkleRoll",    "RHipYawPitch",   "RHipRoll",
      "RHipPitch",      "RKneePitch",    "RAnklePitch",    "RAnkleRoll"};

  for (int i = Joints::headYaw; i < Joints::numOfJoints; i++) {
    if (i == Joints::lHand || i == Joints::rHand) continue;
    joints[i] = getMotor(motorNames[i]);
    joints[i]->setControlPID(Kp, Ki, Kd);
    joints[i]->getPositionSensor()->enable(timeStep);
  }
}

void Nao::run() {
  drawSetup();
  initPosture();
  initModules();
  while (step(timeStep) != -1) {
    readSensors();
    updateModules();
    writeActuators();
    draw();
    logInfo();
  }
}

void Nao::logInfo() {
  float hipP = theJointRequest->angles[Joints::lHipPitch].toDegrees();
    float hipR = theJointRequest->angles[Joints::lHipRoll].toDegrees();
    float cfx = theContactForce->x();
    float cfy = theContactForce->y();
    float zmpx = theZMPData->ZMP.x();
    float zmpy = theZMPData->ZMP.y();
    float pogx = thePoGData->PoG.x();
    float pogy = thePoGData->PoG.y();
    float covx = thePoGData->COV.x();
    float covy = thePoGData->COV.y();
    float bax = theInertialSensorData->angle.x();
    float bay = theInertialSensorData->angle.y();
    cout << bax << ' ' << bay << endl;
    log << theFrameInfo->ftime << " "
        << hipP << " "
        << hipR << " "
        << cfx << " "
        << cfy << " "
        << zmpx << " "
        << zmpy << " "
        << pogx << " "
        << pogy << " "
        << covx << " "
        << covy << " "
        << bax << " "
        << bay << " " << endl;
}

void Nao::draw() {
  //-> Refresh
  drawSetup();

  //-> Plot Measured ZMP
  float x = theZMPData->ZMP.x();
  float y = theZMPData->ZMP.y();
  plot->setColor(0xFF0000);
  plot->fillRectangle(x - 5, y + 5, 10, 10);

  //-> Plot RobotModel CoP
  Vector2f pog = thePoGData->PoG;
  plot->setColor(0x0000FF);
  plot->fillRectangle(pog.x() - 5, pog.y() + 5, 10, 10);

  //-> Draw COV
  int a = thePoGData->COV.x() * thePoGData->ThreshCOV;
  int b = thePoGData->COV.y() * thePoGData->ThreshCOV;
  plot->setColor(0x00FF00);
  plot->drawOval(pog.x(), pog.y(), a, b);
  // plot->drawOval(pog.x(), pog.y(), 10, 10);
}

bool Nao::logWritten() { return false; }

void Nao::drawSetup() {
  //-> Clear Screen
  plot->clear(0x000000);
  //-> Draw Axis
  plot->drawAxis();
  plot->setOrigin(plot->getHeight() / 2, plot->getWidth() / 2);
  plot->setAxis(-100.f, 100.f, -100.f, 100.f);
}

void Nao::readSensors() {
  // Read inertial sensor data
  Vector3f acc = getValuesOfAccelerometer();
  Vector3f gyro = getValuesOfGyro();
  Vector3f rpy = getValuesOfInertialUnit();
  theInertialSensorData->acc << acc.x(), acc.y(), acc.z();
  theInertialSensorData->gyro << gyro.x(), gyro.y(), gyro.z();
  theInertialSensorData->angle << rpy.x(), rpy.y(), rpy.z();

  // Read joint sensor data
  for (int i = 0; i < Joints::numOfJoints; i++) {
    if (i == Joints::lHand || i == Joints::rHand) continue;
    theJointSensorData->angles[i] = joints[i]->getPositionSensor()->getValue();
  }

  // Read fsr sensor data
  Matrix2x4d fsrData = getValuesOfFsr();
  theFsrSensorData->pressures[Legs::left][FsrSensors::fl] = fsrData(0, 0);
  theFsrSensorData->pressures[Legs::left][FsrSensors::fr] = fsrData(0, 1);
  theFsrSensorData->pressures[Legs::left][FsrSensors::br] = fsrData(0, 2);
  theFsrSensorData->pressures[Legs::left][FsrSensors::bl] = fsrData(0, 3);
  theFsrSensorData->pressures[Legs::right][FsrSensors::fl] = fsrData(1, 0);
  theFsrSensorData->pressures[Legs::right][FsrSensors::fr] = fsrData(1, 1);
  theFsrSensorData->pressures[Legs::right][FsrSensors::br] = fsrData(1, 2);
  theFsrSensorData->pressures[Legs::right][FsrSensors::bl] = fsrData(1, 3);

  theFsrSensorData->totals[Legs::left] =
      fsrData(0, 0) + fsrData(0, 1) + fsrData(0, 2) + fsrData(0, 3);
  theFsrSensorData->totals[Legs::right] =
      fsrData(1, 0) + fsrData(1, 1) + fsrData(1, 2) + fsrData(1, 3);
}

void Nao::getSimulatedNao() {
  //! Get real Com position
  // Node *naoNode = getFromDef("NAO");
  // Field *translationField = naoNode->getField("translation");
  // Field *rotationField = naoNode->getField("rotation");
  // const double *transVec = translationField->getSFVec3f();
  // const double *rotVec = rotationField->getSFRotation();
}

void Nao::checkJointAngleLimits(JointAngles &j) {
  for (int i = Joints::firstLegJoint; i < Joints::numOfJoints; i++) {
    j.angles[i] = theJointLimits->limits[i].clamped(j.angles[i]);
  }
}

void Nao::writeActuators() {
  checkJointAngleLimits(*theJointRequest);
  setJoints(*theJointRequest);
}

void Nao::updateModules() {
  //-> Update FrameInfo
  t = getTime();
  theFrameInfo->time = (unsigned)t * 1000;
  theFrameInfo->ftime = t;

  //-> Update ZMP
  theZMPDataProvider.update(*theZMPData);
  //-> Update RobotModel
  theRobotModelProvider.update(*theRobotModel);
  //-> Update PoGData
  thePoGDataProvider.update(*thePoGData);
  //-> Update GroundContactStateProvider
  theGroundContactStateProvider.update(*theGroundContactState);
  //-> Update FallDownStateProvider
  theFallDownStateProvider.update(*theFallDownState);
  //-> Update ContactForceProvider
  theContactForceProvider.update(*theContactForce);

  //-> Update Generator
  generator->update(*theJointRequest);

  //-> Update FitnessProvider
  theFitnessProvider.setStartTime(theFrameInfo->ftime);
  theFitnessProvider.update(*theFitness);

  //-> TEST
}

void Nao::setJoints(const JointAngles &j) {
  /* Set Leg Joints */
  for (int i = Joints::lHipYawPitch; i < Joints::numOfJoints; i++) {
    joints[i]->setPosition(j.angles[i]);
  }
  /* Set Arm Joints (not always need) */
  for (int i = Joints::headYaw; i < Joints::firstLegJoint; i++) {
    if (i == Joints::lHand || i == Joints::rHand) continue;
    joints[i]->setPosition(j.angles[i]);
  }
}

void Nao::setHandsAngle(double angle) {
  // we must activate the 8 phalanx motors
  int j;
  for (j = 0; j < PHALANX_MAX; j++) {
    double clampedAngle = angle;
    if (clampedAngle > maxPhalanxMotorPosition[j])
      clampedAngle = maxPhalanxMotorPosition[j];
    else if (maxPhalanxMotorPosition[j] < minPhalanxMotorPosition[j])
      clampedAngle = minPhalanxMotorPosition[j];

    rPhalanx[j]->setPosition(clampedAngle);
    lPhalanx[j]->setPosition(clampedAngle);
    // wb_motor_set_position(rphalanx[j], clampedAngle);

    // wb_motor_set_position(lphalanx[j], clampedAngle);
  }
}

Eigen::Vector3f Nao::getValuesOfAccelerometer() {
  const double *acc = accelerometer->getValues();
  // printf("--------acc--------\n");
  // printf("acc: [x y z] = [%3.2lf %3.2lf %3.2lf]\n", -acc[0], acc[1], acc[2]);
  return Eigen::Vector3f(-acc[0], acc[1], acc[2]);
}

Vector3f Nao::getValuesOfGyro() {
  const double *vel = gyro->getValues();
  // printf("----------gyro----------\n");
  // printf("angular velocity: [ x y z ] = [%3.4f %3.4f %3.4f]deg/s\n",
  // toDegrees(vel[0]), toDegrees(vel[1]), toDegrees(vel[2]));
  return Vector3f(vel[0], vel[1], vel[2]);
}

Vector3f Nao::getValuesOfGPS() {
  const double *p = gps->getValues();
  // printf("----------gps----------\n");
  // printf("position: [ x y z ] = [%f %f %f]\n", p[0], p[1], p[2]);
  return Vector3f(p[0], p[1], p[2]);
}

Vector3f Nao::getValuesOfInertialUnit() {
  const double *rpy = inertialUnit->getRollPitchYaw();
  // printf("----------inertial unit----------\n");
  // printf("roll/pitch/yaw: = [%3.2f %3.2f %3.2f]deg\n", toDegrees(rpy[0]),
  // toDegrees(rpy[1]), toDegrees(rpy[2]));
  return Vector3f(rpy[0], rpy[1], rpy[2]);
}

Matrix2x4d Nao::getValuesOfFsr() {
  const double *fsv[2] = {fsr[0]->getValues(), fsr[1]->getValues()};
  double l[4], r[4];
  double newtonLeft = 0;
  double newtonRight = 0;

  // The coefficients were calibrated against the real
  // robot so as to obtain realistic sensor values.
  l[0] = fsv[0][2] / 3.4 + 1.5 * fsv[0][0] +
         1.15 * fsv[0][1];  // Left Foot Front Left
  l[1] = fsv[0][2] / 3.4 + 1.5 * fsv[0][0] -
         1.15 * fsv[0][1];  // Left Foot Front Right
  l[2] = fsv[0][2] / 3.4 - 1.5 * fsv[0][0] -
         1.15 * fsv[0][1];  // Left Foot Rear Right
  l[3] = fsv[0][2] / 3.4 - 1.5 * fsv[0][0] +
         1.15 * fsv[0][1];  // Left Foot Rear Left

  r[0] = fsv[1][2] / 3.4 + 1.5 * fsv[1][0] +
         1.15 * fsv[1][1];  // Right Foot Front Left
  r[1] = fsv[1][2] / 3.4 + 1.5 * fsv[1][0] -
         1.15 * fsv[1][1];  // Right Foot Front Right
  r[2] = fsv[1][2] / 3.4 - 1.5 * fsv[1][0] -
         1.15 * fsv[1][1];  // Right Foot Rear Right
  r[3] = fsv[1][2] / 3.4 - 1.5 * fsv[1][0] +
         1.15 * fsv[1][1];  // Right Foot Rear Left

  l[0] /= 1.1419;
  l[1] /= 1.1419;
  l[2] /= 1.1419;
  l[3] /= 1.1419;

  r[0] /= 1.1419;
  r[1] /= 1.1419;
  r[2] /= 1.1419;
  r[3] /= 1.1419;

  for (int i = 0; i < 4; i++) {
    if (l[i] < 0) l[i] = 0;
    if (l[i] > 25) l[i] = 25;
    if (r[i] < 0) r[i] = 0;
    if (r[i] > 25) r[i] = 25;
    newtonLeft += l[i];
    newtonRight += r[i];
  }

  // printf("----------foot sensors----------\n");
  // printf("   left       right\n");
  // printf("+--------+ +--------+\n");
  // printf("|%3.3f  %3.3f| |%3.3f  %3.3f|  front\n", l[0], l[1], r[0], r[1]);
  // printf("|        | |        |\n");
  // printf("|%3.3f  %3.3f| |%3.3f  %3.3f|  back\n", l[3], l[2], r[3], r[2]);
  // printf("+--------+ +--------+\n");
  // printf("total: %g Newtons, %g kilograms\n", newtonLeft + newtonRight,
  // (newtonLeft + newtonRight) / 9.81);

  Matrix2x4d m;
  m << l[0], l[1], l[2], l[3], r[0], r[1], r[2], r[3];
  return m;
}

void Nao::getDevicesName() {
  int n = getNumberOfDevices();
  for (int i = 0; i < n; i++) {
    std::cout << getDeviceNameFromTag(getDeviceTagFromIndex(i)) << std::endl;
  }
}

void Nao::testMotor() {
  std::string motorNames[Joints::numOfJoints] = {
      "HeadYaw",        "HeadPitch",     "LShoulderPitch", "LShoulderRoll",
      "LElbowYaw",      "LElbowRoll",    "LWristYaw",
      "LHand",  // LHand
      "RShoulderPitch", "RShoulderRoll", "RElbowYaw",      "RElbowRoll",
      "RWristYaw",
      "RHand",  // RHand
      "LHipYawPitch",   "LHipRoll",      "LHipPitch",      "LKneePitch",
      "LAnklePitch",    "LAnkleRoll",    "RHipYawPitch",   "RHipRoll",
      "RHipPitch",      "RKneePitch",    "RAnklePitch",    "RAnkleRoll"};
  double t = 0.0;
  for (int id = Joints::headYaw; id < Joints::numOfJoints; id++) {
    if (id == Joints::lHand || id == Joints::rHand) continue;
    for (int j = 0; j < 2; j++) {
      // int id = Joints::headPitch;
      double min_target = joints[id]->getMinPosition();
      double max_target = joints[id]->getMaxPosition();
      double range_target = max_target - min_target;
      while (step(timeStep) != -1) {
        t += dt;
        double target = (max_target - min_target) / 2 * sin(t * pi / 2) +
                        (min_target + max_target) / 2;
        joints[id]->setPosition(target);
        double measurement = joints[id]->getPositionSensor()->getValue();
        if (abs(measurement - target) / range_target > .01 && (j != 0))
          std::cout << motorNames[id] << std::endl;
        if (t > 4) {
          t = 0.0;
          break;
        }
      }
    }
  }
}

void Nao::initModules() {
  while (step(timeStep) != -1) {
    readSensors();
    theZMPDataProvider.update(*theZMPData);
    theRobotModelProvider.update(*theRobotModel);
    thePoGDataProvider.update(*thePoGData);
    ssgenerator->update(*theJointRequest);
    writeActuators();
    if (ssgenerator->finished()) break;
  }
}

void Nao::initPosture() {
  setHandsAngle(0.);
  theJointRequest->angles[Joints::lShoulderPitch] = 90_deg;
  theJointRequest->angles[Joints::lShoulderRoll] = 5_deg;
  theJointRequest->angles[Joints::rShoulderPitch] = 90_deg;
  theJointRequest->angles[Joints::rShoulderRoll] = -5_deg;
  float t = 0.f;
  constexpr float duringT = 2.f;
  constexpr float initHeight = 100.f + 102.9f + 45.19f;

  while (step(timeStep) != -1) {
    t += dt;
    float target =
        initHeight - (t / duringT) * (initHeight - MotionConfig::hipHeight);
    Pose3f targetL =
        Pose3f(Vector3f(0.f, theRobotDimensions->yHipOffset, -target));
    Pose3f targetR =
        Pose3f(Vector3f(0.f, -theRobotDimensions->yHipOffset, -target));
    bool isPossible =
        InverseKinematic::calcLegJoints(targetL, targetR, Vector2f::Zero(),
                                        *theJointRequest, *theRobotDimensions);
    if (isPossible) {
      setJoints(*theJointRequest);
    }
    if (t > duringT) {
      break;
    }

    //-> update ZMP
    readSensors();
    theZMPDataProvider.update(*theZMPData);
  }
}

void Nao::saveSimulation() {}
