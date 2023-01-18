#include "Fitness.h"

#include <iostream>
#include <random>

using namespace std;

Fitness::Fitness() {
  pulNeu = make_unique<Matsuoka2Neu>(Matsuoka2Neu::pulsePattern, _dt);
  xcorr = make_unique<Correlation>(_dt);

  log.open("Logs/Fit.txt");
}

Fitness::~Fitness() { log.close(); }

void Fitness::reset() {
  pulNeu.reset();
  pulNeu = make_unique<Matsuoka2Neu>(Matsuoka2Neu::pulsePattern, _dt);
  _crossZero = false;
  _pulDirection = true;
  _terminalZero = false;

  qD.reset(1.0 / _dt);
  qC.reset(1.0 / _dt);
}

float Fitness::calcFitness(Vector6f param) {
  //-> Reset
  reset();

  //-> check Params Tu and Tv cannot be zeros.
  const float Tu = param[0];
  const float Tv = param[1];
  if (abs(Tu) < 1e-4 || abs(Tv) < 1e-4) {
    //-> The divisor cannot be zero
    _fitness = -10;
    return _fitness;
  }
  //-> Inital fitness;
  float fitness = -1;
  //-> Set Neuron parameters.
  pulNeu->setParam(param);

  //-> Score: maxY
  const int N = xcorr->getN();
  const float T = xcorr->getT();
  float maxY = pulNeu->getYAbsMax(N * T);

  //-> LOG
  _maxY = maxY;

  if (isinf(maxY) || isnan(maxY)) {
    //-> Not a number
    _fitness = -10.f;
    return _fitness;
  }
  if (abs(maxY) < 1e-2) {
    _fitness = -5.f;
    return _fitness;  //-> is Zero
  }
  if (abs(maxY) > 100) {
    _fitness = -2.f;
    return _fitness;  //-> too Large MaxY
  }

  //-> Integral Neuron
  vector<float> samples;
  samples.resize(N);
  float tU1V1 = 0;
  float tU2V2 = 0;
  float tU1V2 = 0;
  float tU2V1 = 0;
  const float capturedValue = 1;
  for (int i = 0; i < N; i++) {
    pulNeu->integrate();
    float y, u1, v1, u2, v2;
    tU1V1 = abs(u1 - v1);
    tU2V2 = abs(u2 - v2);
    tU1V2 = abs(u1 - v2);
    tU2V1 = abs(u2 - v1);
    tie(y, u1, v1, u2, v2) = pulNeu->yuv();
    y = y / maxY;
    // const float y = pulNeu->y() / maxY;
    samples[i] = y;

    //-> cross zeros point
    if (y < 0) {
      _crossZero = true;
    }
    //-> Check pulse direction
    qD.insert(y);
    if (qD.full() && qD.size() > 0) {
      if (qD.sign() < 0) {
        _pulDirection = false;
        // break;  // Wrong moving direction.
      }
      qD.reset(0);
    }
    //-> check terminal zero
    // const float tValue = 0.01;
    qC.insert(y);
    if (qC.full()) {
      terminalE = qC.average();
    }

    // log << samples[i] << endl;
  }
  _terminalE = terminalE;

  if (!_crossZero || !_pulDirection) {
    _fitness = -20;
    return _fitness;
  }

  if (tU1V1 > capturedValue || tU2V2 > capturedValue || tU1V2 > capturedValue ||
      tU2V1 > capturedValue) {
    _fitness = -30;
    return _fitness;
  }

  float score = xcorr->calc(samples);
  if (score >= 1.0) {
    score = -score;
  }
  const float fTerminalE = 1.0 / exp(abs(terminalE) / 10.0);

  _scorr = xcorr->getSCorr();
  _xcorr = xcorr->getXCorr();
  fitness = score * fTerminalE;
  // fitness = score;
  _fitness = fitness;

  return fitness;
}

void Fitness::test() {}

void Fitness::info() {
  cout << "{\n";
  cout << "\tfitness: " << _fitness << endl;
  cout << "\tmaxY: " << _maxY << endl;
  cout << "\tSCorr: " << _scorr << endl;
  cout << "\tXCorr: " << _xcorr << endl;
  cout << "\tterminalE: " << _terminalE << endl;
  cout << "\tcrosszero: " << _crossZero << endl;
  cout << "\tpuldirection: " << _pulDirection << endl;
  cout << "}" << endl;
}
