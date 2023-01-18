#pragma once

#include <Eigen/Eigen>
#include <tuple>

#include "Rk4.h"

using Vector6f = Eigen::Matrix<float, 6, 1>;

class Matsuoka2Neu {
 public:
  enum CPGPattern { pulsePattern, OscPattern };

 public:
  Matsuoka2Neu(CPGPattern type, float dt);
  void setInit(float u1, float v1, float u2, float v2);
  void integrate();
  void integrateUntil(float maxtime);
  Rk4 getIntegrator() const { return rk; }
  float getYAbsMax(float maxtime);
  void setFeed(float f1, float f2) {
    this->feed1 = f1;
    this->feed2 = f2;
  }
  void setParam(const Vector6f &p);
  std::tuple<float, float, float, float, float> yuv();

 private:
  void initParamsPulse();
  void initParamsOsc();
  void setParams(float Tu, float Tv, float s0, float b, float wfe, float feed1,
                 float feed2, float K_feed1, float K_feed2);
  void reset();

 public:
  float Tu;
  float Tv;
  float s0;
  float b;
  float wfe;
  float feed1;  // feed1 = desire - measure;
  float feed2;  // feed2 = -feed1;
  float K_feed1;
  float K_feed2;

  float dt;
  // float t;

  RK::state_type y0;
  Rk4 rk;
  RK::state_type args;
};