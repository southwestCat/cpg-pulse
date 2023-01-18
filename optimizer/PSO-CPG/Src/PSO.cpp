#include "PSO.h"

#include <ctime>
#include <chrono>
#include <thread>
#include <iostream>

#include <iomanip>
#include<sys/types.h>
#include<sys/ioctl.h>
#include<unistd.h>
#include<termios.h>

using namespace std;

PSO::PSO()
{
    //-> Init params
    c1 = 0.1f;
    c2 = 0.1f;
    w = 0.8f;

    //-> pIndex
    particleIndex = 0;
    iterIndex = 0;
    gbest_obj = OBJMIN;

    e.seed(rd());
    u0 = make_unique<uniform_real_distribution<float>>(-1.0, 1.0);
    u1 = make_unique<uniform_real_distribution<float>>(0, 1.0);
    n = make_unique<normal_distribution<float>>(0, 1.0);

    // log.open("Logs/PSO.log", std::ios::app | std::out);
    log.open("Logs/PSO.log");
}

PSO::~PSO()
{
    log.close();
}

Vector6f PSO::randomX()
{
    Vector6f x;
    while (true)
    {
        for (int i = 0; i < 5; i++)
        {
            // x[i] =  * 10.0;
            //->
            x[i] = (*u0)(e)*10.0;
        }
        if (x[0] != 0 && x[1] != 0)
            break;
    }
    x[5] = 1.0; //-> Do not optimized K.
    return x;
}

Vector6f PSO::randomV()
{
    Vector6f v;
    for (int i = 0; i < 5; i++)
    {
        // x[i] =  * 10.0;
        //->
        v[i] = (*n)(e)*0.1;
    }
    v[5] = 0;
    return v;
}

void PSO::initParams()
{
    auto t1 = chrono::steady_clock::now();

    X[0] = (Vector6f() << 0.6f, 0.4f, 1.f, 1.8f, 1.6f, 1.f).finished();
    for (size_t i = 1; i < nParticles; i++)
    {
        X[i] = randomX();
        V[i] = randomV();
    }

    thread t[nThreads];

    auto f1 = [&](int b)
    {
        Fitness f;
        for (size_t i = 0; i < nTasks; i++)
        {
            int Index = i + b;
            float obj = f.calcFitness(X[Index]);

            pbest[Index] = X[Index].eval();
            pbest_obj[Index] = obj;
        }
    };

    for (size_t i = 0; i < nThreads; i++)
    {
        t[i] = thread(f1, i * nTasks);
    }
    for (size_t i = 0; i < nThreads; i++)
    {
        t[i].join();
    }

    updateGBest();

    auto t2 = chrono::steady_clock::now();
    double drS = chrono::duration<double>(t2 - t1).count();
    cout << "Initial PSO params using: " << drS << " s." << endl;
}

void PSO::update()
{
    for (int it = 0; it < nIter; it++)
    {
        //-> 1. Update Particles
        updateParticles();

        //-> 2. Update PBest
        thread t[nThreads];

        auto f1 = [&](int b)
        {
            Fitness f;
            for (size_t i = 0; i < nTasks; i++)
            {
                int Index = i + b;
                float obj = f.calcFitness(X[Index]);
                if (obj > pbest_obj[Index])
                {
                    pbest[Index] = X[Index];
                    pbest_obj[Index] = obj;
                }
            }
        };

        for (size_t i = 0; i < nThreads; i++)
        {
            t[i] = thread(f1, i * nTasks);
        }
        for (size_t i = 0; i < nThreads; i++)
        {
            t[i].join();
        }

        //-> 3. Update GBest
        updateGBest();

        checkGBest();
        if (progTerminate)
            break;

        cout << it << " -> | " << gbest_obj << " | "
            << gbest[0] << ", "
            << gbest[1] << ", "
            << gbest[2] << ", "
            << gbest[3] << ", "
            << gbest[4] << ", "
            << gbest[5] << endl;

        if ((it+1) % 10 == 0)
            updateAnswer();
    }
}

void PSO::updateParticles()
{
    float r1 = (*u1)(e);
    float r2 = (*u1)(e);

    auto f1 = [&](int b)
    {
        for (size_t i = 0; i < nTasks; i++)
        {
            V[i + b] = w * V[i + b] + c1 * r1 * (pbest[i + b] - X[i + b]) + c2 * r2 * (gbest - X[i + b]);
            X[i + b] = X[i + b] + V[i + b];
        }
    };

    thread t[nThreads];

    for (size_t i = 0; i < nThreads; i++)
    {
        t[i] = thread(f1, i * nTasks);
    }
    for (size_t i = 0; i < nThreads; i++)
    {
        t[i].join();
    }
}

void PSO::updateGBest()
{
    auto f1 = [&](int iP, int b)
    {
        tGBestObj[iP] = OBJMIN;
        for (size_t i = 0; i < nTasks; i++)
        {
            if (pbest_obj[i + b] > tGBestObj[iP])
            {
                tGBest[iP] = pbest[i + b];
                tGBestObj[iP] = pbest_obj[i + b];
            }
        }
    };

    thread t[nThreads];
    for (size_t i = 0; i < nThreads; i++)
    {
        t[i] = thread(f1, i, i * nTasks);
    }
    for (size_t i = 0; i < nThreads; i++)
    {
        t[i].join();
    }

    for (size_t i = 0; i < nThreads; i++)
    {
        if (tGBestObj[i] > gbest_obj)
        {
            gbest = tGBest[i].eval();
            gbest_obj = tGBestObj[i];
        }
    }
}

void PSO::progressBar(int n, int total)
{
    struct winsize size;
    ioctl(STDIN_FILENO, TIOCGWINSZ, &size);
    const int per = n  * size.ws_col / total - 6;
    cout << "[" << setw(3) << (int)n*100/total << "%]";
    for (int j = 0; j < per; j++)
    {
        cout << "#";
    }
    cout << "\r";
    fflush(stdout);
}

void PSO::updateAnswer()
{
    cout << "-- Updating answer..." << endl;
    Fitness f;
    int nAns = 0;
    Vector6f ans = Vector6f::Zero();
    for (size_t i = 0; i < nParticles; i++)
    {
        float obj = f.calcFitness(X[i]);
        if (obj > 0)
        {
            nAns++;
            ans += X[i];
        }
        if (i % 100 == 0)
        {
            progressBar(i, nParticles);
        }
    }
    if (nAns > 0)
    {
        ans /= (float)nAns;
    }
    cout << "\n-- Answer is: ";
    cout << ans[0] << ", ";
    cout << ans[1] << ", ";
    cout << ans[2] << ", ";
    cout << ans[3] << ", ";
    cout << ans[4] << ", ";
    cout << ans[5] << endl;

    log << ans[0] << ", ";
    log << ans[1] << ", ";
    log << ans[2] << ", ";
    log << ans[3] << ", ";
    log << ans[4] << ", ";
    log << ans[5] << endl;
}

void PSO::updateAnswerThreads()
{
    array<Vector6f, nThreads> ans;
    auto f1 = [&](int b, Vector6f &ans)
    {
        Vector6f sum = Vector6f::Zero();
        int cnt = 0;
        for (size_t i = 0; i < nTasks; i++)
        {
            if (pbest_obj[i + b] > 0)
            {
                cnt++;
                sum += pbest[i + b];
            }
        }
        if (cnt > 0)
            ans = sum / (float)cnt;
        else
            ans = Vector6f::Zero();
    };
    thread t[nThreads];
    for (size_t i = 0; i < nThreads; i++)
    {
        t[i] = thread(f1, i * nTasks, ref(ans[i]));
    }
    for (size_t i = 0; i < nThreads; i++)
    {
        t[i].join();
    }

    answer = Vector6f::Zero();
    int nAns = 0;
    for_each(ans.cbegin(), ans.cend(), [&](Vector6f n)
             {
        // answer += n;
        if (n != Vector6f::Zero())
        {
            nAns++;
            answer += n;
        } });
    if (nAns > 0)
        answer /= (float)nAns;
    else
        answer = Vector6f::Zero();
}

void PSO::checkGBest()
{
    Fitness f;
    const float obj = f.calcFitness(gbest);
    cout << "Checked: [ " << obj << " ]" << endl;
    if (obj != gbest_obj)
    {
        progTerminate = true;
        cout << "gbest not correct !!!\n";
        log << "{" << endl;
        log << "gbest:" << endl;
        log << "| " << gbest_obj << " | ";
        log << gbest[0] << ", ";
        log << gbest[1] << ", ";
        log << gbest[2] << ", ";
        log << gbest[3] << ", ";
        log << gbest[4] << ", ";
        log << gbest[5] << ", ";
        log << " --> [ " << obj << " ]" << endl;
        log << "pbest:" << endl;
        Fitness fc;
        for (size_t i = 0; i < nParticles; i++)
        {
            const float ob = fc.calcFitness(pbest[i]);

            if (ob != pbest_obj[i])
            {
                log << "| " << pbest_obj[i] << " | ";
                log << pbest[i][0] << ", ";
                log << pbest[i][1] << ", ";
                log << pbest[i][2] << ", ";
                log << pbest[i][3] << ", ";
                log << pbest[i][4] << ", ";
                log << pbest[i][5] << ", ";
                log << " --> [ " << ob << " ]" << endl;
            }
        }
        log << "}" << endl
            << endl;
    }
}

void PSO::test()
{
    for (int i = 0; i < (int)nParticles; i++)
    {
        if (i % 100 == 0)
        {
            progressBar(i, nParticles);
            this_thread::sleep_for(chrono::milliseconds(50));
        }
    }
    cout << endl;
}
