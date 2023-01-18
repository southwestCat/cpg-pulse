#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Module/Queue.h"

#include <memory>
#include <fstream>

class ZMPData;
class PoGData;
class FrameInfo;
class JointRequest;
class Matsuoka2Neu;
class CPGGeneratorOutput;
class RobotDimensions;
class ContactForce;
class Fitness;

class CPGGenerator
{
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

    USES_REPRESENTATION(Fitness);
    
    MODIFIES_REPRESENTATION(CPGGeneratorOutput);

private:
    enum class State
    {
        none,
        recovery,
        standing,
        startPul,
        inPul,
        endPul,
        reset
    };

    State pStat;

    float dt;
    float contactForceStartTime;
    bool contactForceReach = false;
    Vector2f force;
    std::unique_ptr<Matsuoka2Neu> pulNeuX;
    std::unique_ptr<Matsuoka2Neu> pulNeuY;


    // Matsuoka2Neu* pulNeu;  //! Pulse Pattern
    // Matsuoka2Neu* OscNeu;  //! Oscillate Pattern
    std::ofstream log;

    Vector3f WPB;
    Vector3f WPLF;
    Vector3f WPRF;
    Vector3f lWPLF;
    Vector3f lWPRF;

    Queue qx;
    Queue qy;

    const float MAXCOSTTIME = 5.0;
    const float RECOVTIME = 1.0;
    const float tValue = 1;
    const float tdValue = 0.01;
    const float Kx = 20.f;
    const float Ky = 20.f;

    float ymaxX = 0;
    float ymaxY = 0;
    float tStartRecv;

    Vector2f rCOM;

private:
    void makeJoints(JointRequest &j);
    void recover(JointRequest &j);
    void standing();
    void startPulse();
    void inPulse(JointRequest &j);
    void endPulse();
    void init();
    bool checkContactForce();
    bool isTerminatedX(float y);
    bool isTerminatedY(float y);
    void setCoM(float x, float y, JointRequest &j);
};
