#pragma once

struct GroundContactState
{
    bool contact;

    void reset()
    {
        contact = false;
    }
};
