#pragma once

#include <sys/time.h>
#include <string>

class FrameInfo
{
public:
    int getTimeSince(unsigned timeStamp) const;
    float getTimeSince(float timestamp) const;
    static std::string getCurrentSystemTime()
    {
        struct timeval now_time;
        gettimeofday(&now_time, NULL);
        time_t tt = now_time.tv_sec;
        tm *temp = localtime(&tt);

        char buf[64];
        snprintf(buf, 64, "%04d-%02d-%02d %02d:%02d:%02d:%02d",
                 temp->tm_year + 1900,
                 temp->tm_mon + 1,
                 temp->tm_mday,
                 temp->tm_hour,
                 temp->tm_min,
                 temp->tm_sec,
                 (int)now_time.tv_usec / 10000);

        std::string str(buf);
        return str;
    }

    unsigned time = 0;
    float ftime = 0.f;
};

inline int FrameInfo::getTimeSince(unsigned timeStamp) const
{
    return static_cast<int>(time - timeStamp);
}

inline float FrameInfo::getTimeSince(float timestamp) const
{
    return (ftime - timestamp);
}
