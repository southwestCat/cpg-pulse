#pragma once

#include <vector>

class Queue
{
public:
    Queue() = default;
    ~Queue();
    Queue(size_t size);
    void reset(size_t size);
    size_t size() { return _size; }
    float max();
    bool empty() { return _d.empty(); }
    bool full() { return _d.size() == _size; }
    void insert(float d);
    float average();
    float variance();
    float sum();
    int sign(); 
    float absmax();
    float deltaMaxMin();

private:
    std::vector<float> _d;
    size_t _size = 0;
    size_t _i = 0;
};