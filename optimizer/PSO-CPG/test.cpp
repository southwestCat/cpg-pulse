#include <iostream>
#include <array>
#include <thread>
#include <chrono>
#include <mutex>
#include <fstream>
#include <Eigen/Eigen>

#include <iomanip>
#include<sys/types.h>
#include<sys/ioctl.h>
#include<unistd.h>
#include<termios.h>

#include "Fitness.h"
#include "PSO.h"
#include "Gaussian.h"

using namespace std;
using namespace Eigen;
using Vector6f = Eigen::Matrix<float, 6, 1>;

int main()
{
    Vector6f v;
    v << 3.10412, 0.424391, 9.28835, 9.31659, -2.99902, 1;
    
    Fitness fit;

    const float obj = fit.calcFitness(v);
    cout << obj << endl;
    fit.info();
}

int main1()
{
    // auto t1 = chrono::steady_clock::now();
    // PSO pso;
    // Fitness f;
    // for (int i = 0; i < 100; i++)
    // {
    //     float fitness = f.calcFitness(pso.test());
    //     cout << i << ": " << fitness << endl;
    // }
    // auto t2 = chrono::steady_clock::now();
    // double drS = chrono::duration<double>(t2-t1).count();
    // cout << "duration: " << drS << endl;

    /////////////////////

    mutex mtx;

    auto t1 = chrono::steady_clock::now();
    PSO pso;

    const size_t nThreads = 16;
    const size_t nTasks = 125;
    const size_t nParticles = nThreads * nTasks;

    array<float, nParticles> d;
    array<Vector6f, nParticles> p;
    for (size_t i = 0; i < d.size(); i++)
    {
        d[i] = -1;
    }

    auto f1 = [&](size_t b, Fitness &f)
    {
        // Fitness f;
        for (size_t i = 0; i < nTasks; i++)
        {
            mtx.lock();
            Vector6f v = pso.getRandomX();
            mtx.unlock();
            p[i + b] = v;
            d[i + b] = f.calcFitness(v);
            if (isnan(d[i + b]) || isinf(d[i + b]))
            {
                cout << "####" << endl;
                cout << v << endl;
                cout << "#####" << endl;
            }
        }
    };

    thread t[nThreads];
    Fitness fitn[nThreads];
    for (size_t i = 0; i < nThreads; i++)
    {
        t[i] = thread(f1, i * nTasks, ref(fitn[i]));
    }
    for (size_t i = 0; i < nThreads; i++)
    {
        t[i].join();
    }

    auto t2 = chrono::steady_clock::now();
    double drS = chrono::duration<double>(t2 - t1).count();
    cout << "duration: " << drS << endl;

    cout << "check results..." << endl;

    Fitness fit;
    for (size_t i = 0; i < d.size(); i++)
    {
        const float fitnn = fit.calcFitness(p[i]);
        if (fitnn != d[i])
        {
            cout << i << " " << fitnn << " " << d[i] << endl;
            break;
        }
    }

    return 0;
}