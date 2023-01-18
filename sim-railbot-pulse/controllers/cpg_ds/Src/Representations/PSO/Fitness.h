#pragma once

struct Fitness
{
    float fitness;

    float costtime;
    const float MAXCOSTTIME = 20.f;
    float totalControlError;
    bool fallDownState; //-> True: fall; False: not fall

    bool terminate = false;

    void reset()
    {
        terminate = false;
    }
};