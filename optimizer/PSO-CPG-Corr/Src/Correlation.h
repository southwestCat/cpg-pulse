#pragma once

#include <fftw3.h>

#include <Eigen/Eigen>
#include <vector>
#include <fstream>

using Vector2d = Eigen::Matrix<double, 2, 1>;

class Correlation {
 public:
  Correlation(float dt);
  ~Correlation();

  float calc(const std::vector<float> &samples);
  const int getN() const { return N; }
  const float getT() const { return T; }
  float getSCorr() { return selfCorr; }
  float getXCorr() { return _xcorr; }
  void info();

 private:
  void calcSignature();

 private:
  float _dt;
  const float T = _dt;
  const int N = 512;
  const int N_2 = N / 2 + 1;
  double selfCorr = 0;
  float _xcorr = 0;
  float score = 0;
  int zeroIndex = 0;

 private:
  double *samples;
  double *correlation;
  fftw_complex *spectrum;
  fftw_plan fft;
  fftw_plan ifft;
  std::vector<Vector2d> signature;
  std::ofstream logT;
  std::ofstream logS;
};