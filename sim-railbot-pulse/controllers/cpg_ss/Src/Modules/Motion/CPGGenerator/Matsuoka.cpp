#include "Matsuoka.h"

#include <algorithm>
#include <cmath>

using namespace std;

// #define MATSUOKA_FUNCTION_USE_MAX

Matsuoka2Neu::Matsuoka2Neu(CPGPattern type, float dt) {
  this->dt = dt;

  if (type == pulsePattern) {
    initParamsPulse();
  } else if (type == OscPattern) {
    initParamsOsc();
  }

  //-> Set Params
  // setParams();

  //-> Set Matsuoka Function
  rk.func = [&](const RK::state_type &y,
                RK::state_type &args) -> RK::state_type {
    float u1 = y[0];
    float v1 = y[1];
    float u2 = y[2];
    float v2 = y[3];
#ifdef MATSUOKA_FUNCTION_USE_MAX
    float y1 = std::max(0.f, u1);
    float y2 = std::max(0.f, u2);
#else
    float y1 = u1;
    float y2 = u2;
#endif
    RK::state_type dydt(4);

    dydt[0] = (s0 - u1 - b * v1 - wfe * y2 + K_feed1 * feed1) / Tu;
    dydt[1] = (y1 - v1) / Tv;
    dydt[2] = (s0 - u2 - b * v2 - wfe * y1 + K_feed2 * feed2) / Tu;
    dydt[3] = (y2 - v2) / Tv;

    return dydt;
  };

  //-> Set Initial Value
  setInit(0.f, 0.f, 0.f, -0.2f);
}

std::tuple<float, float, float, float, float> Matsuoka2Neu::yuv() {
  RK::state_type state = rk.states.back();
  const float u1 = state[0];
  const float v1 = state[1];
  const float u2 = state[2];
  const float v2 = state[3];
#ifdef MATSUOKA_FUNCTION_USE_MAX
  float y1 = std::max(0.f, u1);
  float y2 = std::max(0.f, u2);
#else
  float y1 = u1;
  float y2 = u2;
#endif

  float y_ = y2 - y1;
  return make_tuple(y_, u1, v1, u2, v2);
}

void Matsuoka2Neu::initParamsPulse() {
  Tu = 0.6f;
  Tv = 0.4f;
  s0 = 1.f;
  b = 1.8f;
  wfe = 1.6f;
  feed1 = 0.f;
  feed2 = 0.f;
  K_feed1 = 1.f;
  K_feed2 = 1.f;
}

void Matsuoka2Neu::initParamsOsc() {
  Tu = 0.2f;
  Tv = 0.4f;
  s0 = 1.f;
  b = 1.8f;
  wfe = 1.6;
  feed1 = 0.f;
  feed2 = 0.f;
  K_feed1 = 1.f;
  K_feed2 = 1.f;
}

void Matsuoka2Neu::setInit(float u1, float v1, float u2, float v2) {
  //-> Clear Vector
  RK::state_type().swap(y0);
  //-> Set value
  y0.push_back(u1);
  y0.push_back(v1);
  y0.push_back(u2);
  y0.push_back(v2);

  rk.initial(y0);
}

void Matsuoka2Neu::setParams(float Tu, float Tv, float s0, float b, float wfe,
                             float feed1, float feed2, float K_feed1,
                             float K_feed2) {
  this->Tu = Tu;
  this->Tv = Tv;
  this->s0 = s0;
  this->b = b;
  this->wfe = wfe;
  this->feed1 = feed1;
  this->feed2 = feed2;
  this->K_feed1 = K_feed1;
  this->K_feed2 = K_feed2;
}

void Matsuoka2Neu::setParam(const Vector6f &p) {
  this->Tu = p[0];
  this->Tv = p[1];
  this->s0 = p[2];
  this->b = p[3];
  this->wfe = p[4];
  this->K_feed1 = p[5];
  this->K_feed2 = p[5];
}

void Matsuoka2Neu::integrate() { rk.integrate(dt, args); }

void Matsuoka2Neu::integrateUntil(float maxtime) {
  float t = 0.f;
  while (t < maxtime) {
    integrate();
    t += dt;
  }
}

float Matsuoka2Neu::getYAbsMax(float maxtime) {
  integrateUntil(maxtime);

  float ymax = 0.f;
  for (auto state : rk.states) {
    const float u1 = state[0];
    const float u2 = state[2];
#ifdef MATSUOKA_FUNCTION_USE_MAX
  float y1 = std::max(0.f, u1);
  float y2 = std::max(0.f, u2);
#else
  float y1 = u1;
  float y2 = u2;
#endif
    const float y = y2 - y1;
    if (abs(y) > ymax) {
      ymax = abs(y);
    }
  }

  reset();

  return ymax;
}

void Matsuoka2Neu::reset() {
  rk.reset();
  rk.initial(y0);
}
