#include "ZMPDataProvider.h"
#include "Tools/Math/Eigen.h"

#include <iostream>
using namespace std;

void ZMPDataProvider::update(ZMPData &zmp)
{
    //-> Force
    float lfl = theFsrSensorData->pressures[Legs::left][FsrSensors::fl];
    float lfr = theFsrSensorData->pressures[Legs::left][FsrSensors::fr];
    float lbl = theFsrSensorData->pressures[Legs::left][FsrSensors::bl];
    float lbr = theFsrSensorData->pressures[Legs::left][FsrSensors::br];

    float rfl = theFsrSensorData->pressures[Legs::right][FsrSensors::fl];
    float rfr = theFsrSensorData->pressures[Legs::right][FsrSensors::fr];
    float rbl = theFsrSensorData->pressures[Legs::right][FsrSensors::bl];
    float rbr = theFsrSensorData->pressures[Legs::right][FsrSensors::br];

    float ltotal = lfl + lfr + lbl + lbr;
    float rtotal = rfl + rfr + rbl + rbr;
    float total = ltotal + rtotal;

    //-> Position
    const Vector2f Pl = {0.f, theRobotDimensions->yHipOffset};
    const Vector2f Plfl = theRobotDimensions->leftFsrPositions[FsrSensors::fl] + Pl;
    const Vector2f Plfr = theRobotDimensions->leftFsrPositions[FsrSensors::fr] + Pl;
    const Vector2f Plbl = theRobotDimensions->leftFsrPositions[FsrSensors::bl] + Pl;
    const Vector2f Plbr = theRobotDimensions->leftFsrPositions[FsrSensors::br] + Pl;

    const Vector2f Pr = {0.f, -theRobotDimensions->yHipOffset};
    const Vector2f Prfl = theRobotDimensions->rightFsrPositions[FsrSensors::fl] + Pr;
    const Vector2f Prfr = theRobotDimensions->rightFsrPositions[FsrSensors::fr] + Pr;
    const Vector2f Prbl = theRobotDimensions->rightFsrPositions[FsrSensors::bl] + Pr;
    const Vector2f Prbr = theRobotDimensions->rightFsrPositions[FsrSensors::br] + Pr;

    Vector2f mZMP = (lfl * Plfl + lfr * Plfr + lbl * Plbl + lbr * Plbr +
                     rfl * Prfl + rfr * Prfr + rbl * Prbl + rbr * Prbr) /
                    total;
    float alpha = zmp.alpha;
    //-> Filter
    zmp.ZMP = alpha * zmp.ZMP + (1.f - alpha) * mZMP;
    //-> No Filter
    // zmp.ZMP = mZMP;
}