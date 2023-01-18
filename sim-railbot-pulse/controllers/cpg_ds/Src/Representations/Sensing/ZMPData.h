#pragma once

#include "Tools/Math/Eigen.h"

struct ZMPData
{
    Vector2f ZMP = Vector2f(0.f, 0.f); // In Body Frame

    // const float singleSupportAlpha = 0.6f;
    // const float doubleSupportAlpha = 0.9f;

    const float singleSupportAlpha = 0.4f;
    const float doubleSupportAlpha = 0.4f;

    float alpha = 0.8f;

    void reset()
    {
        ZMP = Vector2f::Zero();
    }
};