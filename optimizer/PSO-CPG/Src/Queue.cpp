#include "Queue.h"
#include <algorithm>

using namespace std;

Queue::Queue(size_t size)
{
    reset(size);
}

Queue::~Queue()
{
    std::vector<float>().swap(_d);
}

void Queue::reset(size_t size)
{
    _size = size;
    _i = 0;
    std::vector<float>().swap(_d);
}

void Queue::insert(float d)
{
    //-> Cannot insert to a empty queue.
    if (_size == 0)
        return;
    
    if (_d.size() < _size)
    {
        _d.emplace_back(d);
    }
    else
    {
        if (_i == _size)
            _i = 0;
        _d.at(_i) = d;
        _i++;
    }
}

float Queue::absmax()
{
    if (empty())
        return 0.f;
    float _m = 0.f;
    for_each(_d.cbegin(), _d.cend(), [&](float n){
        if (abs(n) > _m)
            _m = abs(n);
    });
    return _m;
}

float Queue::sum()
{
    float s = 0.f;
    for_each(_d.cbegin(), _d.cend(), [&s](float n)
    {
        s += n; 
    });
    return s;
}

float Queue::average()
{
    return sum() / _size;
}

float Queue::variance()
{
    float a = average();
    float svar = 0.f;
    for_each(_d.cbegin(), _d.cend(), [&](float n){
        svar += (n-a)*(n-a);
    });
    return svar / _size;
}

float Queue::deltaMaxMin()
{   
    float _max = *max_element(_d.cbegin(), _d.cend());
    float _min = *min_element(_d.cbegin(), _d.cend());
    return _max - _min;
}

int Queue::sign()
{
    int s = 0;
    for_each(_d.cbegin(), _d.cend(), [&](float n){
        s += n > 0 ? 1 : -1;
    });
    return s;
}
