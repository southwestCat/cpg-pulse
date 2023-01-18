#pragma once

#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Display.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>

#include "Tools/Math/Eigen.h"
#include "Tools/RobotParts/Joints.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Module/DebugDraw.h"

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Sensing/ZMPData.h"
#include "Representations/Sensing/ContactForce.h"

#include "Modules/Sensing/ZMPDataProvider.h"
#include "Modules/Sensing/RobotModelProvider/RobotModelProvider.h"
#include "Modules/Sensing/PoGDataProvider/PoGDataProvider.h"
#include "Modules/Motion/CPGGenerator/CPGGenerator.h"
#include "Modules/Sensing/GroundContactStateProvider/GroundContactStateProvider.h"
#include "Modules/Sensing/FallDownStateProvider/FallDownStateProvider.h"
#include "Modules/Optimizer/FitnessProvider.h"
#include "Modules/Sensing/ContactForceProvider/ContactForceProvider.h"
#include "Modules/Motion/SingleSupportGenerator/SingleSupportGenerator.h"

#include <fstream>
#include <memory>

#define PHALANX_MAX 8

class Nao : public webots::Supervisor, Blackboard
{
public:
    Nao();
    ~Nao();
    void run();

private:
    int timeStep = 32;
    webots::Accelerometer *accelerometer;
    webots::Camera *cameraTop, *cameraBottom;
    webots::DistanceSensor *distanceSensor[2];
    webots::GPS *gps;
    webots::Gyro *gyro;
    webots::InertialUnit *inertialUnit;
    webots::LED *led;
    webots::Motor *lPhalanx[PHALANX_MAX];
    webots::Motor *rPhalanx[PHALANX_MAX];
    webots::Motor *joints[Joints::numOfJoints];
    webots::TouchSensor *fsr[2];
    webots::TouchSensor *lFootLBumper;
    webots::TouchSensor *lFootRBumper;
    webots::TouchSensor *rFootLBumper;
    webots::TouchSensor *rFootRBumprt;
    webots::LED *leds[7];
    webots::Display *display;
    webots::Emitter *emitter;
    webots::Receiver *receiver;

    std::unique_ptr<DebugDraw> plot;
    // DebugDraw *plot = nullptr;

    // Time step using in interpolation
    double dt;
    double t = 0.0;

    // Tuned PID Parameters
    double Kp = 150;
    double Ki = 30;
    double Kd = .1;
    double maxPhalanxMotorPosition[PHALANX_MAX];
    double minPhalanxMotorPosition[PHALANX_MAX];

    //-> Log File
    std::ofstream log;

    //! Controller
    std::unique_ptr<CPGGenerator> generator;
    std::unique_ptr<SingleSupportGenerator> ssgenerator;

    // Representations
    REQUIRES_REPRESENTATION(JointLimits);
    REQUIRES_REPRESENTATION(RobotDimensions);

    MODIFIES_REPRESENTATION(FrameInfo);
    MODIFIES_REPRESENTATION(FsrSensorData);
    MODIFIES_REPRESENTATION(InertialSensorData);
    MODIFIES_REPRESENTATION(JointSensorData);
    MODIFIES_REPRESENTATION(JointRequest);
    MODIFIES_REPRESENTATION(ZMPData);
    MODIFIES_REPRESENTATION(RobotModel);
    MODIFIES_REPRESENTATION(PoGData);
    MODIFIES_REPRESENTATION(GroundContactState);
    MODIFIES_REPRESENTATION(FallDownState);
    MODIFIES_REPRESENTATION(Fitness);
    MODIFIES_REPRESENTATION(ContactForce);

    // Providers
    ZMPDataProvider theZMPDataProvider;
    RobotModelProvider theRobotModelProvider;
    PoGDataProvider thePoGDataProvider;
    GroundContactStateProvider theGroundContactStateProvider;
    FallDownStateProvider theFallDownStateProvider;
    FitnessProvider theFitnessProvider;
    ContactForceProvider theContactForceProvider;

private:
    void initDevice();
    void updateModules();
    void readSensors(); // read measured joint angles from motor
    void getSimulatedNao();
    void writeActuators();
    bool logWritten();
    void setJoints(const JointAngles &j); // set calculated joint angles to motor
    void checkJointAngleLimits(JointAngles &j);
    void setHandsAngle(double angle);
    void initPosture();
    void initModules();
    void drawSetup();
    void draw();
    void saveSimulation();
    void logInfo();

    /* Update Module function */

    /* Read sensor function */
    Vector3f getValuesOfAccelerometer();
    Vector3f getValuesOfGyro();
    Vector3f getValuesOfGPS();
    Vector3f getValuesOfInertialUnit();
    Matrix2x4d getValuesOfFsr();

    /* Tool Functions */
    void getDevicesName();
    void testMotor();

    //-> Supervisor field
private:
    const double INITIAL_TRANS[3] = {0, 0, 0.333327};
    const double INITIAL_ROT[4] = {0, 0, 1, 0};
    void setInitialTransAndRot();
};