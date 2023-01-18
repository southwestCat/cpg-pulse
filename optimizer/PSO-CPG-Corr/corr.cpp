#include <fftw3.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>

using namespace std;
using Vector2d = Eigen::Matrix<double, 2, 1>;

int main() {
  auto t1 = chrono::steady_clock::now();

  const int Fs = 100;
  float T = 1.f / Fs;
  const int N = 512;
  array<float, N> t;
  vector<float> f_sin;
  vector<Vector2d> signature;

  for (int i = 0; i < N; i++) {
    t[i] = (float)i * T;
    f_sin.push_back(sin(2.0 / 3.0 * M_PI * t[i]));
  }

  double *samples = fftw_alloc_real(N);
  memset(samples, 0, sizeof(double) * N);
  fftw_complex *spectrum = fftw_alloc_complex(N / 2 + 1);
  double *correlation = fftw_alloc_real(N);

  fftw_plan fft = fftw_plan_dft_r2c_1d(N, samples, spectrum, FFTW_MEASURE);
  fftw_plan ifft = fftw_plan_dft_c2r_1d(N, spectrum, correlation, FFTW_MEASURE);

  //-> signature signal
  for (int i = 0; i < N; i++) {
    samples[i] = sin(2.0 / 3.0 * M_PI * (float)i * T);
  }
  fftw_execute(fft);

  signature.resize(N / 2 + 1);

  for (size_t i = 0; i < signature.size(); i++) {
    signature[i] = Vector2d(spectrum[i][0], -spectrum[i][1]);
    spectrum[i][0] =
        spectrum[i][0] * spectrum[i][0] + spectrum[i][1] * spectrum[i][1];
    spectrum[i][1] = 0;
  }

  fftw_execute(ifft);

  double bestCorr = 0;
  for (size_t i = 0; i < N; i++) {
    if (abs(correlation[i] > bestCorr)) {
      bestCorr = abs(correlation[i]);
    }
  }

  cout << "Best corr: " << sqrt(bestCorr) / N << endl;

  //-> recording signal
  for (int i = 0; i < N; i++) {
    // samples[i] = sin(2.0 / 3.0 * M_PI * (float)i * T);
    // samples[i] = sin(M_PI * (float)i * T);
    samples[i] = sin(4.0 * M_PI * (float)i * T);
  }
  fftw_execute(fft);
  for (size_t i = 0; i < N / 2 + 1; i++) {
    const double spectrumi0 = spectrum[i][0];
    spectrum[i][0] =
        spectrumi0 * signature[i][0] - spectrum[i][1] * signature[i][1];
    spectrum[i][1] =
        spectrum[i][1] * signature[i][0] + spectrumi0 * signature[i][1];
  }
  fftw_execute(ifft);
  double xcorr = 0;
  for (size_t i = 0; i < N; i++) {
    if (abs(correlation[i]) > xcorr) {
      xcorr = abs(correlation[i]);
    }
  }
  cout << "xcorr: " << sqrt(xcorr) / N << endl;
  cout << "score: " << sqrt(xcorr) / N / (sqrt(bestCorr) / N) << endl;

  fftw_destroy_plan(ifft);
  fftw_destroy_plan(fft);
  fftw_free(correlation);
  fftw_free(spectrum);
  fftw_free(samples);

  auto t2 = chrono::steady_clock::now();
  double dr = chrono::duration<double>(t2 - t1).count();
  cout << "cost time: " << dr << "s." << endl;
  return 0;
}
