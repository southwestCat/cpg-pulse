#pragma once

#pragma once

#include <Eigen/Eigen>
#include <array>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <random>
#include <vector>

#include "Fitness.h"

using Vector6f = Eigen::Matrix<float, 6, 1>;

class PSO {
 public:
  PSO();
  ~PSO();

  void initParams();
  void update();
  void updateAnswer();

  //-> costtime
  //-> cumulateUErr: control input cumulative error
  //-> terminal state error, com return to the initial position
  //-> cumulateAErr: cumulative bodyangle and gyro error
  //-> fall: robot falldown
  //-> direction: complianct motion is the same direction with the applied force
  // std::function<float(float costtime, float cumulateUErr, float terminalErr,
  // float cumulateAErr, bool fall, bool direction)> func;

  // std::vector<float> fitness;
  // std::vector<Vector6f> params;

 private:
  float c1;
  float c2;
  float w;

  int particleIndex;
  int iterIndex;
  // bool updateGBest = false;
  bool paramInit = false;

  std::random_device rd;
  std::default_random_engine e;
  std::unique_ptr<std::uniform_real_distribution<float>> u0;
  std::unique_ptr<std::uniform_real_distribution<float>> u1;
  std::unique_ptr<std::normal_distribution<float>> n;

 private:
  //->
  const int nIter = 10000;
  const float OBJMIN = -1000;
  static const size_t nThreads = 16;
  static const size_t nTasks = 1000;
  static const size_t nParticles = nThreads * nTasks;
  std::array<Vector6f, nParticles> X;
  std::array<Vector6f, nParticles> V;
  std::array<Vector6f, nParticles> pbest;
  Vector6f gbest;
  std::array<float, nParticles> pbest_obj;
  float gbest_obj;

  std::array<Vector6f, nThreads> tGBest;
  std::array<float, nThreads> tGBestObj;

  Vector6f answer;

  std::ofstream log;

  bool progTerminate = false;

  std::mutex mtx;

 private:
  Vector6f randomX();
  Vector6f randomV();
  void updateGBest();
  void updateParticles();
  void updateAnswerThreads();
  void checkGBest();
  void progressBar(int now, int total);

 public:
  void test();
  Vector6f getRandomX() { return randomX(); }
  Vector6f getAnswer() { return answer; }
};
