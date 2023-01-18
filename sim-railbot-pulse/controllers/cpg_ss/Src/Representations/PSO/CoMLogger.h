#pragma once

#include <vector>
#include "Tools/Math/Eigen.h"

struct CoMLogger
{
    std::vector<Vector2f> com;
    std::vector<float> timestamp;

    void reset()
    {
        std::vector<Vector2f>().swap(com);
        std::vector<float>().swap(timestamp);
    }
};