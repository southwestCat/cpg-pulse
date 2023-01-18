#include "GroundContactStateProvider.h"

void GroundContactStateProvider::update(GroundContactState &g)
{
    float totalL = theFsrSensorData->totals[Legs::left];
    float totalR = theFsrSensorData->totals[Legs::right];

    g.contact = true;

    if (totalL < 10.f
    || totalR < 10.f)
    {
        g.contact = false;
    }
}