#pragma once

#include "CoMLogger.h"

class CPGGeneratorOutput
{
public:
    CoMLogger comlog;
    float startTime;
    float finishTime;
    bool finish = false;
    float control;
    float terminal;
    float forceX;
    bool startOuput = false;
    float comx;
    float comy;

    bool inPul = false;

    void reset()
    {
        comlog.reset();
        finishTime = 0;
        finish = false;
        control = 0.f;
        terminal = 0.f;
        startOuput = false;
        inPul = false;
    }
};