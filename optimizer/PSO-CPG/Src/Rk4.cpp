#include "Rk4.h"
#include <iostream>

void Rk4::initial(RK::state_type &initial)
{
    t = 0.f;

    RK::state_type().swap(y0);
    RK::state_type().swap(times);
    std::vector<RK::state_type>().swap(states);

    for (auto i : initial)
    {
        y0.push_back(i);
    }
    
    times.push_back(t);
    states.push_back(y0);
}

void Rk4::integrate(float dt, RK::state_type &args)
{
    t += dt;
    times.push_back(t);

    //-> k1
    RK::state_type yn = states.back();
    RK::state_type k1 = func(yn, args);
    //-> k2
    RK::state_type yk1;
    for (size_t i = 0; i < k1.size(); i++)
    {
        yk1.push_back(yn[i] + dt / 2.f * k1[i]);
    }
    RK::state_type k2 = func(yk1, args);
    //-> k3
    RK::state_type yk2;
    for (size_t i = 0; i < k2.size(); i++)
    {
        yk2.push_back(yn[i] + dt / 2.f * k2[i]);
    }
    RK::state_type k3 = func(yk2, args);
    //-> k4
    RK::state_type yk3;
    for (size_t i = 0; i < k3.size(); i++)
    {
        yk3.push_back(yn[i] + dt * k3[i]);
    }
    RK::state_type k4 = func(yk3, args);
    //-> yn+1
    RK::state_type yn1;
    for (size_t i = 0; i < yn.size(); i++)
    {
        yn1.push_back(yn[i] + dt / 6.f * (k1[i] + 2.f * k2[i] + 2.f * k3[i] + k4[i]));
    }

    states.push_back(yn1);
}

void Rk4::reset()
{
    t = 0.f;
    RK::state_type().swap(y0);
    RK::state_type().swap(times);
    std::vector<RK::state_type>().swap(states);
}
