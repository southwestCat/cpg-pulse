#pragma once

#include "Tools/Math/Eigen.h"

struct PoGData
{
    Vector2f PoG = {0.f, 0.f};
    Vector2f COV = {0.1f, 0.1f};

    const float ThreshCOV = 80.f;
    const float InitCOVRatio = 0.1f;

    const float ax = 0.0454f;
    const float bx = 19.6487f;

    const float ay = 0.5877f;
    const float by = 0.0053f;
};
