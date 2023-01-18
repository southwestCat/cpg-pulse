#pragma once

#include <cassert>
#include <map>
#include <memory>
#include <vector>

#define REQUIRES_REPRESENTATION(representation)            \
  const representation *the##representation =              \
      Blackboard::getInstance().the##representation.get(); \
  const bool update##representation = true;

#define USES_REPRESENTATION(representation)                \
  const representation *the##representation =              \
      Blackboard::getInstance().the##representation.get(); \
  const bool update##representation = false;

#define MODIFIES_REPRESENTATION(representation)            \
  representation *the##representation =                    \
      Blackboard::getInstance().the##representation.get(); \
  const bool update##representation = false;

class FrameInfo;
class FsrSensorData;
class InertialSensorData;
class JointRequest;
class JointSensorData;
class RobotModel;
class RobotDimensions;
class JointLimits;
class MassCalibration;
class ZMPData;
class PoGData;
class FallDownState;
class GroundContactState;
class Fitness;
class CoMLogger;
class ContactForce;
class CPGGeneratorOutput;
class SupportState;

class Blackboard {
 public:
  Blackboard();
  ~Blackboard();

  static Blackboard &getInstance();
  static void setInstance(Blackboard *instance);
  float dt() const { return timeStamp_; }

  float timeStamp_ = 0.01f;

  std::unique_ptr<FrameInfo> theFrameInfo;
  std::unique_ptr<FsrSensorData> theFsrSensorData;
  std::unique_ptr<InertialSensorData> theInertialSensorData;
  std::unique_ptr<JointRequest> theJointRequest;
  std::unique_ptr<JointSensorData> theJointSensorData;
  std::unique_ptr<RobotModel> theRobotModel;
  std::unique_ptr<RobotDimensions> theRobotDimensions;
  std::unique_ptr<JointLimits> theJointLimits;
  std::unique_ptr<MassCalibration> theMassCalibration;
  std::unique_ptr<ZMPData> theZMPData;
  std::unique_ptr<PoGData> thePoGData;
  std::unique_ptr<FallDownState> theFallDownState;
  std::unique_ptr<GroundContactState> theGroundContactState;
  std::unique_ptr<Fitness> theFitness;
  std::unique_ptr<CoMLogger> theCoMLogger;
  std::unique_ptr<ContactForce> theContactForce;
  std::unique_ptr<CPGGeneratorOutput> theCPGGeneratorOutput;
  std::unique_ptr<SupportState> theSupportState;

 public:
  std::map<std::string, bool> updatedRepresentation;
  std::map<std::string, bool> updatedConfig;
  bool exists(std::string representation);

 private:
  enum MapType { representationMap, configMap };

  // void initMap();
  // void initRepresentationMap();
  // void initConfigMap();
  void setRMap(std::string representation);
  void setCMap(std::string config);
  void insert(std::string string, MapType type = representationMap);
};