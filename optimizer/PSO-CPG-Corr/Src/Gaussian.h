#pragma once

#include <cmath>

class Gaussian {
 public:
  Gaussian(float mu, float sig) : _mu(mu), _sig(sig) {}

  float calc(float x) const {
    return 1.0 / sqrt(2.0 * M_PI) / _sig *
           exp(-(x - _mu) * (x - _mu) / 2 / _sig / _sig);
  }

  float max() const { return calc(_mu); }

  float calcN(float x) const {
    float r = x;
    // return calc(x) / max();
    if (x < _mu) r = -_radio * x + (_radio + 1) * _mu;
    return calc(r) / max();
  }

  float &mu() { return _mu; }
  const float &mu() const { return _mu; }
  float &maxRange() { return _maxRange; }
  const float &maxRange() const { return _maxRange; }
  float &radio() { return _radio; }
  const float &radio() const { return _radio; }

 private:
  float _mu;
  float _sig;
  float _maxRange = 20.f;
  float _radio = 1.f;
};