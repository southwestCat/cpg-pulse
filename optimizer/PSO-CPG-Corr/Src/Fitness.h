#pragma once

#include <Eigen/Eigen>
#include <array>
#include <fstream>
#include <memory>

#include "Correlation.h"
#include "Gaussian.h"
#include "Matsuoka.h"
#include "Queue.h"

using Vector6f = Eigen::Matrix<float, 6, 1>;

class Fitness {
 public:
  Fitness();
  ~Fitness();
  float calcFitness(Vector6f param);
  void reset();
  void test();
  void info();

 private:
  enum class Stat { none, terminate, wrongDirection, overtime };
  const float _dt = 0.01;

  //-> Desired Generated CPG Patter
  const float MAXCOSTTIME = 5.f;
  Queue qC;  // Control Queue
  Queue qD;  // Direction Queue

  std::unique_ptr<Matsuoka2Neu> pulNeu;
  std::unique_ptr<Correlation> xcorr;

 private:
  float terminalE;
  float _terminalE;
  float _maxY;
  float _fitness;
  float _scorr;
  float _xcorr;
  bool _crossZero = false;
  bool _pulDirection = true;
  bool _terminalZero = false;
  std::ofstream log;
};
