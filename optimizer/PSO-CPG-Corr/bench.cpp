#include <iostream>
#include <array>
#include <chrono>
#include "PSO.h"

using namespace std;
using namespace chrono;

int main()
{
    PSO pso;
    int nTimes = 10;
    cout << "TEST BENCH START..." << endl;
    auto t1 = steady_clock::now();
    for (int i = 0; i < nTimes; i++)
    {
        pso.initParams();
    }
    auto t2 = steady_clock::now();
    double dr = duration<double>(t2 - t1).count();
    cout << "TEST " << nTimes << " times." << endl;
    cout << "Total cost time is " << dr << endl;
    cout << "Average Initial Time: " << dr / (double)nTimes << endl;
}