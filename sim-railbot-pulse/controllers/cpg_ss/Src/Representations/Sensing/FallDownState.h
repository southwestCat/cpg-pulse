#pragma once

class FallDownState
{
public:
    bool fall; //-> True: fall; False: not fall
    void reset()
    {
        fall = false;
    }
};
