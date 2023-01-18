#pragma once

#include <vector>
#include <functional>

namespace RK
{
    typedef std::vector<float> state_type;
}

struct Rk4
{
    Rk4() = default;

    void initial(RK::state_type &initial);
    void integrate(float dt, RK::state_type &args);
    void reset();

    float t = 0.f;

    std::function<RK::state_type(const RK::state_type &, RK::state_type &)> func;
    RK::state_type y0;
    RK::state_type times;
    std::vector<RK::state_type> states;
};
