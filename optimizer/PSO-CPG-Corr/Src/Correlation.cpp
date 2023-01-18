#include "Correlation.h"

#include <cmath>
#include <iostream>
#include <mutex>

using namespace std;
recursive_mutex _mtx;
#define SYNC lock_guard<recursive_mutex> _lock(_mtx)

Correlation::Correlation(float dt) : _dt(dt) {
  samples = fftw_alloc_real(N);
  memset(samples, 0, sizeof(double) * N);
  correlation = fftw_alloc_real(N);
  spectrum = fftw_alloc_complex(N_2);

  logT.open("Logs/CorrelationT.txt");
  logS.open("Logs/CorrelationS.txt");

  SYNC;
  fft = fftw_plan_dft_r2c_1d(N, samples, spectrum, FFTW_MEASURE);
  ifft = fftw_plan_dft_c2r_1d(N, spectrum, correlation, FFTW_MEASURE);

  calcSignature();
}

Correlation::~Correlation() {
  logT.close();
  logS.close();
  SYNC;
  fftw_destroy_plan(ifft);
  fftw_destroy_plan(fft);
  fftw_free(correlation);
  fftw_free(spectrum);
  fftw_free(samples);
}

void Correlation::calcSignature() {
  //-> 0. template signal
  const float A = 2.31975;
  const float tau = 1.91387;
  const float w = 2.50499;
  for (int i = 0; i < N; i++) {
    const float t = (float)i * T;
    // samples[i] = sin(2.0 / 3.0 * M_PI * t);
    samples[i] = A * exp(-tau * t) * sin(w * t);
  }
  //-> 1. fft signal
  fftw_execute(fft);
  //-> 1. Eliminate DC component
  spectrum[0][0] = 0;
  spectrum[0][1] = 0;
  //-> 3. ifft signal
  fftw_execute(ifft);
  //-> 4. Normalized
  float maxSample = 0;
  for (int i = 0; i < N; i++) {
    samples[i] = correlation[i];
    if (abs(samples[i]) > maxSample) {
      maxSample = abs(samples[i]);
    }
  }
  for (int i = 0; i < N; i++) {
    samples[i] /= maxSample;
    // logT << samples[i] << endl;
  }
  //-> 5. fft signature
  fftw_execute(fft);
  //-> 6. correlating self
  signature.resize(N_2);
  for (int i = 0; i < N_2; i++) {
    signature[i] = Vector2d(spectrum[i][0], -spectrum[i][1]);
    spectrum[i][0] =
        spectrum[i][0] * spectrum[i][0] + spectrum[i][1] * spectrum[i][1];
    spectrum[i][1] = 0;
  }
  fftw_execute(ifft);
  double bestCorr = 0;
  for (int i = 0; i < N; i++) {
    if (abs(correlation[i] > bestCorr)) {
      bestCorr = abs(correlation[i]);
    }
  }
  selfCorr = sqrt(bestCorr) / N;
}

float Correlation::calc(const vector<float> &s) {
  assert(s.size() == (size_t)N);

  score = 0;

  //-> 0. sampling signal
  for (int i = 0; i < N; i++) {
    samples[i] = static_cast<double>(s[i]);
  }
  //-> 1. fft signal
  fftw_execute(fft);
  //-> 2. Eliminate DC component
  spectrum[0][0] = 0;
  spectrum[0][1] = 0;
  //-> 3. ifft signal
  fftw_execute(ifft);
  //-> 4. Normalized
  float maxSample = 0;
  for (int i = 0; i < N; i++) {
    samples[i] = correlation[i];
    if (abs(samples[i]) > maxSample) {
      maxSample = abs(samples[i]);
    }
  }
  for (int i = 0; i < N; i++) {
    samples[i] /= maxSample;
    // logS << samples[i] << endl;
  }
  //-> 5. fft normalized sampling signal
  fftw_execute(fft);
  //-> 6. correlating sampling
  for (int i = 0; i < N_2; i++) {
    const double spectrumi0 = spectrum[i][0];
    spectrum[i][0] =
        spectrumi0 * signature[i][0] - spectrum[i][1] * signature[i][1];
    spectrum[i][1] =
        spectrum[i][1] * signature[i][0] + spectrumi0 * signature[i][1];
  }
  fftw_execute(ifft);
  double xcorr = 0;
  for (int i = 0; i < N; i++) {
    if (abs(correlation[i] > xcorr)) {
      xcorr = abs(correlation[i]);
    }
  }
  _xcorr = static_cast<float>(sqrt(xcorr) / N);
  score = sqrt(xcorr) / N / selfCorr;
  return score;
}

void Correlation::info() {
  cout << "{" << endl;
  cout << "\tT: " << T << endl;
  cout << "\tSelfCorr: " << selfCorr << endl;
  cout << "\tcorr: " << _xcorr << endl;
  cout << "\tscore: " << score << endl;
  cout << "}" << endl;
}
