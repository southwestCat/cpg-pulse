#include "Blackboard.h"

#include <iostream>

#include "Representations/Configuration/JointLimits.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Motion/SupportState.h"
#include "Representations/PSO/CPGGeneratorOutput.h"
#include "Representations/PSO/CoMLogger.h"
#include "Representations/PSO/Fitness.h"
#include "Representations/Sensing/ContactForce.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/PoGData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/ZMPData.h"

#ifndef CLASS2STRING
#define CLASS2STRING(class_name) #class_name
#endif

using namespace std;

static Blackboard *theInstance = nullptr;

Blackboard::Blackboard() {
  theInstance = this;

  theFrameInfo = make_unique<FrameInfo>();
  theFsrSensorData = make_unique<FsrSensorData>();
  theInertialSensorData = make_unique<InertialSensorData>();
  theJointRequest = make_unique<JointRequest>();
  theJointSensorData = make_unique<JointSensorData>();
  theRobotModel = make_unique<RobotModel>();
  theRobotDimensions = make_unique<RobotDimensions>();
  theJointLimits = make_unique<JointLimits>();
  theMassCalibration = make_unique<MassCalibration>();
  theZMPData = make_unique<ZMPData>();
  thePoGData = make_unique<PoGData>();
  theFallDownState = make_unique<FallDownState>();
  theGroundContactState = make_unique<GroundContactState>();
  theFitness = make_unique<Fitness>();
  theCoMLogger = make_unique<CoMLogger>();
  theContactForce = make_unique<ContactForce>();
  theCPGGeneratorOutput = make_unique<CPGGeneratorOutput>();
  theSupportState = make_unique<SupportState>();
}

Blackboard::~Blackboard() { theInstance = nullptr; }

Blackboard &Blackboard::getInstance() { return *theInstance; }

void Blackboard::setInstance(Blackboard *instance) { theInstance = instance; }

void Blackboard::insert(std::string string, MapType type) {
  // Default std::map::insert will ignore same element.
  // this exist() is not necessary, but maybe some help, e.g.
  // want to inert representationMap, but inert configMap
  if (!exists(string)) {
    if (type == configMap) {
      updatedConfig.insert(std::pair<std::string, bool>(string, false));
    } else if (type == representationMap) {
      updatedRepresentation.insert(std::pair<std::string, bool>(string, false));
    } else {
      std::cout << "No this type map" << std::endl;
    }
  }
}

bool Blackboard::exists(std::string representation) {
  return (updatedRepresentation.find(representation) !=
          updatedRepresentation.end()) ||
         (updatedConfig.find(representation) != updatedConfig.end());
}

void Blackboard::setRMap(std::string representation) {
  if (!exists(representation))
    updatedRepresentation.insert(
        std::pair<std::string, bool>(representation, false));
  updatedRepresentation[representation] = true;
}

void Blackboard::setCMap(std::string config) {
  if (!exists(config))
    updatedConfig.insert(std::pair<std::string, bool>(config, false));
  updatedConfig[config] = true;
}
