#include "ContactForceProvider.h"
#include "Representations/Sensing/ContactForce.h"
#include "Representations/Sensing/ZMPData.h"
#include "Representations/Sensing/PoGData.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Modules/Motion/config.h"
#include "Tools/Math/Constants.h"
#include <iostream>

using namespace std;

void ContactForceProvider::update(ContactForce &f)
{
    //-> Hip height as com height
    const float h = MotionConfig::hipHeight;
    //-> Mass
    const float m = theMassCalibration->totalMass / 1000.f;
    //-> Get ZMP data and PoG data
    const Vector2f &zmp = theZMPData->ZMP;
    const Vector2f &pog = thePoGData->PoG;
    const Vector2f &cov = thePoGData->COV * thePoGData->ThreshCOV;

    const float zmpx = zmp.x();
    const float zmpy = zmp.y();
    const float pogx = pog.x();
    const float pogy = pog.y();

    const float covx = cov.x();
    const float covy = cov.y();

    //-> Update force in x direction
    if (zmpx < pogx + covx && zmpx > pogx - covx)
    {
        f.x() = 0.f;
    }
    else
    {
        f.x() = Kx * m * Constants::g_1000 * (zmpx - pogx) / h;
    }
    //-> Update force in y direction
    if (zmpy < pogy + covy && zmpy > pogy - covy)
    {
        f.y() = 0.f;
    }
    else
    {
        f.y() = Ky * m * Constants::g_1000 * (zmpy - pogy) / h;
    }
}