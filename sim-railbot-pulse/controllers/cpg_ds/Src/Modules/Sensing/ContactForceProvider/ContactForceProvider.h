#pragma once

#include "Tools/Module/Blackboard.h"   

class ContactForce;
class FrameInfo;
class ZMPData;
class PoGData;
class MassCalibration;

class ContactForceProvider
{
public:
    void update(ContactForce &f);

private:
    const float Kx = 1.f;
    const float Ky = 1.f;

private:
    REQUIRES_REPRESENTATION(FrameInfo);
    REQUIRES_REPRESENTATION(ZMPData);
    REQUIRES_REPRESENTATION(PoGData);
    REQUIRES_REPRESENTATION(MassCalibration);
};