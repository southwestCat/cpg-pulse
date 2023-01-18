#include "FallDownStateProvider.h"
#include <iostream>

using namespace std;

#define DEG(x) (x/3.1415926*180.0)

void FallDownStateProvider::update(FallDownState &f)
{
    f.fall = false;

    float angleX = DEG(theInertialSensorData->angle.x()) - 90.0;
    float angleY = DEG(theInertialSensorData->angle.y());

    float xBias = abs(angleX);
    float yBias = abs(angleY);

    float bias = max(xBias, yBias);

    if ((bias > 45.f)
        && (!theGroundContactState->contact))
    {
        f.fall = true;
    }
}