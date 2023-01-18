#include <sys/ioctl.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Eigen>
#include <array>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
#include <tuple>

#include "Correlation.h"
#include "Fitness.h"
#include "Gaussian.h"
#include "PSO.h"

using namespace std;
using namespace Eigen;
using Vector6f = Eigen::Matrix<float, 6, 1>;

int main11() {
  const float T = 0.01;
  const int N = 512;
  Correlation c(T);

  vector<float> samples;
  samples.resize(N);
  const float A = 2.31975;
  const float tau = 1.91387;
  const float w = 2.50499;
  for (size_t i = 0; i < samples.size(); i++) {
    const float t = (float)i * T;
    samples[i] = A * exp(-tau * t) * sin(w * t);
    // samples[i] = sin(4.0 * M_PI * t);
  }
  c.calc(samples);
  c.info();

  return 0;
}

int main() {
  Vector6f v;
  v << 2.2937, 0.212537, -3.09476, 4.90564, 0.995818, 1;

  Fitness fit;

  const float obj = fit.calcFitness(v);
  cout << obj << endl;
  fit.info();

  return 0;
}

int main1() {
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
  for (size_t i = 0; i < d.size(); i++) {
    d[i] = -1;
  }

  auto f1 = [&](size_t b, Fitness &f) {
    // Fitness f;
    for (size_t i = 0; i < nTasks; i++) {
      mtx.lock();
      Vector6f v = pso.getRandomX();
      mtx.unlock();
      p[i + b] = v;
      d[i + b] = f.calcFitness(v);
      if (isnan(d[i + b]) || isinf(d[i + b])) {
        cout << "####" << endl;
        cout << v << endl;
        cout << "#####" << endl;
      }
    }
  };

  thread t[nThreads];
  Fitness fitn[nThreads];
  for (size_t i = 0; i < nThreads; i++) {
    t[i] = thread(f1, i * nTasks, ref(fitn[i]));
  }
  for (size_t i = 0; i < nThreads; i++) {
    t[i].join();
  }

  auto t2 = chrono::steady_clock::now();
  double drS = chrono::duration<double>(t2 - t1).count();
  cout << "duration: " << drS << endl;

  cout << "check results..." << endl;

  Fitness fit;
  for (size_t i = 0; i < d.size(); i++) {
    const float fitnn = fit.calcFitness(p[i]);
    if (fitnn != d[i]) {
      cout << i << " " << fitnn << " " << d[i] << endl;
      break;
    }
  }

  return 0;
}