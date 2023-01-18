#include "CPGGenerator.h"
#include "Matsuoka.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/ZMPData.h"
#include "Representations/Sensing/PoGData.h"
#include "Representations/PSO/CoMLogger.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Sensing/ContactForce.h"
#include "Representations/PSO/CPGGeneratorOutput.h"
#include "Representations/PSO/Fitness.h"
#include "Modules/Motion/config.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Motion/InverseKinematic.h"
#include <iostream>

using namespace std;

CPGGenerator::CPGGenerator(float dt) : dt(dt)
{
    init();

    // log.open("Logs/CPGGenerator.log");
}

void CPGGenerator::init()
{
    pulNeuX.reset();
    pulNeuY.reset();

    pulNeuX = make_unique<Matsuoka2Neu>(Matsuoka2Neu::pulsePattern, dt);
    pulNeuY = make_unique<Matsuoka2Neu>(Matsuoka2Neu::pulsePattern, dt);

    pStat = State::standing;
    contactForceReach = false;

    qx.reset(1.0 / dt);
    qy.reset(1.0 / dt);
}

CPGGenerator::~CPGGenerator()
{
    // log.close();
}

bool CPGGenerator::checkContactForce()
{
    if (pStat != State::standing)
    {
        return true;
    }
    else
    {
        theCPGGeneratorOutput->startTime = theFrameInfo->ftime;
        if (theContactForce->norm() > 0.f)
        {
            contactForceReach = true;
            force = *theContactForce;
            theCPGGeneratorOutput->forceX = force.x();
            // return true;
        }
        else
        {
            if (!contactForceReach)
                contactForceStartTime = theFrameInfo->ftime;
            return false;
        }

        if (contactForceReach)
        {
            if (theFrameInfo->getTimeSince(contactForceStartTime) > 0.1)
            {
                return true;
            }
            if (abs(theContactForce->x()) > abs(force.x()))
                force = *theContactForce;
            return false;
        }
    }
    return true;
}

void CPGGenerator::update(JointRequest &j)
{
    return;
    if (!checkContactForce())
    {
        theCPGGeneratorOutput->startOuput = false;
        theCPGGeneratorOutput->comx = 0;
        theCPGGeneratorOutput->comy = 0;
        return;
    }

    theCPGGeneratorOutput->startOuput = true;
    theCPGGeneratorOutput->inPul = false;

    switch (pStat)
    {
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
        break;
    default:
        break;
    }
}

void CPGGenerator::makeJoints(JointRequest &j)
{
}

void CPGGenerator::standing()
{
    pStat = State::startPul;
}

void CPGGenerator::startPulse()
{
    Vector6f param = (Vector6f() << 3.10412, 0.424391, 9.28835, 9.31659, -2.99902, 1).finished();
    
    pulNeuX->setParam(param);
    pulNeuX->setInit(0,0,0,-0.2);
    ymaxX = pulNeuX->getYAbsMax(MAXCOSTTIME);
    pulNeuX->setInit(1.26932, 1.26933, 1.26932, 1.26933);

    pulNeuY->setParam(param);
    pulNeuY->setInit(0,0,0,-0.2);
    ymaxY = pulNeuY->getYAbsMax(MAXCOSTTIME);
    pulNeuY->setInit(1.26932, 1.26933, 1.26932, 1.26933);

    pStat = State::inPul;
}

// void CPGGenerator::startPulse()
// {
//     Vector6f param = (Vector6f() << 0.6, 0.4, 1.0, 1.8, 1.6, 1).finished();
    
//     pulNeuX->setParam(param);
//     pulNeuX->setInit(0.23027, 0.227971, 0.224276, 0.226575);
//     ymaxX = pulNeuX->getYAbsMax(MAXCOSTTIME);

//     pulNeuY->setParam(param);
//     pulNeuY->setInit(0.23027, 0.227971, 0.224276, 0.226575);
//     ymaxY = pulNeuY->getYAbsMax(MAXCOSTTIME);

//     pStat = State::inPul;
// }


void CPGGenerator::inPulse(JointRequest &j)
{
    theCPGGeneratorOutput->inPul = true;
    //-> Set feedback X
    float feedX = -theContactForce->x();
    // feedX = 0;
    pulNeuX->setFeed(feedX, -feedX);
    pulNeuX->integrate();
    float yX, u1X,v1X, u2X, v2X;
    tie(yX, u1X,v1X, u2X, v2X) = pulNeuX->yuv();
    yX = yX / ymaxX * Kx;
    // yX = 0;

    //-> Set feedback Y
    float feedY = -theContactForce->y();
    // feedY = 0;
    pulNeuY->setFeed(feedY, -feedY);
    pulNeuY->integrate();
    float yY, u1Y, v1Y, u2Y, v2Y;
    tie(yY, u1Y, v1Y, u2Y, v2Y) = pulNeuY->yuv();
    yY = yY / ymaxY * Ky;
    // yY = 0;
    // log << yY << ", " << u1Y << ", " << v1Y << ", " << u2Y << ", " << v2Y << endl;
    theCPGGeneratorOutput->comx = yX;
    theCPGGeneratorOutput->comy = yY;

    if (isTerminatedX(yX) && isTerminatedY(yY))
    {
        pStat = State::endPul;
    }


    //-> WPB
    WPB = Vector3f(yX, yY, MotionConfig::hipHeight);
    WPLF = Vector3f(0, theRobotDimensions->yHipOffset, 0);
    WPRF = Vector3f(0, -theRobotDimensions->yHipOffset, 0);

    rCOM = Vector2f(yX, yY);

    Vector3f BPLF = -WPB + WPLF;
    Vector3f BPRF = -WPB + WPRF;

    Pose3f targetL(BPLF);
    Pose3f targetR(BPRF);

    JointRequest _j;
    bool isPossible = InverseKinematic::calcLegJoints(targetL, targetR, Vector2f::Zero(), _j, *theRobotDimensions);
    if (isPossible)
    {
        for (int i = Joints::firstLegJoint; i <= Joints::rAnkleRoll; i++)
        {
            j.angles[i] = _j.angles[i];
        }
    }
}

void CPGGenerator::endPulse()
{    
    qx.reset(1.0 / dt);
    qy.reset(1.0 / dt);

    tStartRecv = theFrameInfo->ftime;

    pStat = State::recovery;
    theCPGGeneratorOutput->finish = true;
    theCPGGeneratorOutput->finishTime = theFrameInfo->ftime;
}

void CPGGenerator::reset()
{
    init();
}

bool CPGGenerator::isTerminatedX(float y)
{
    bool r = false;
    qx.insert(y);
    if (qx.full())
    {
        if (qx.absmax() < tValue || qx.deltaMaxMin() < tdValue) r = true;
    }

    return r;
}

bool CPGGenerator::isTerminatedY(float y)
{
    bool r = false;
    qy.insert(y);
    if (qy.full())
    {
        if (qy.absmax() < tValue || qy.deltaMaxMin() < tdValue) r = true;
    }
    return r;
}

void CPGGenerator::recover(JointRequest &j)
{
    float t = theFrameInfo->getTimeSince(tStartRecv);
    float x = (1 - t/RECOVTIME) * rCOM.x();
    float y = (1 - t/RECOVTIME) * rCOM.y();

    theCPGGeneratorOutput->comx = x;
    theCPGGeneratorOutput->comy = y;
    
    setCoM(x, y, j);
    if (t > RECOVTIME)
    {
        setCoM(0, 0, j);
        pStat = State::standing;
    }
}

void CPGGenerator::setCoM(float x, float y, JointRequest &j)
{
    //-> WPB
    WPB = Vector3f(x, y, MotionConfig::hipHeight);
    WPLF = Vector3f(0, theRobotDimensions->yHipOffset, 0);
    WPRF = Vector3f(0, -theRobotDimensions->yHipOffset, 0);

    Vector3f BPLF = -WPB + WPLF;
    Vector3f BPRF = -WPB + WPRF;

    Pose3f targetL(BPLF);
    Pose3f targetR(BPRF);

    JointRequest _j;
    bool isPossible = InverseKinematic::calcLegJoints(targetL, targetR, Vector2f::Zero(), _j, *theRobotDimensions);
    if (isPossible)
    {
        for (int i = Joints::firstLegJoint; i <= Joints::rAnkleRoll; i++)
        {
            j.angles[i] = _j.angles[i];
        }
    }
}
