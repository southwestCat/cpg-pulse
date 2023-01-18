#include "Fitness.h"

#include <random>
#include <iostream>

using namespace std;

Fitness::Fitness()
{
    pulNeu = make_unique<Matsuoka2Neu>(Matsuoka2Neu::pulsePattern, _dt);
    //-> overshoot
    gau = make_unique<Gaussian>(MAXSHOOT, 1);
    gau->radio() = 0.2f;
    gau->maxRange() = MAXCOSTTIME;
    // costtime
    gauT = make_unique<Gaussian>(DEFINEDHALFPERIOD, 0.6);
    gauT->radio() = 2.f;
    gauT->maxRange() = MAXCOSTTIME;

    w = {0, 2.0, 1.0, 2.0};
    float wsum = 0.f;
    for_each(w.cbegin(), w.cend(), [&wsum](float n)
             { wsum += n; });
    for_each(w.begin(), w.end(), [&wsum](float &n)
             { n /= wsum; });

    qD.reset(1.0 / _dt);
    qC.reset(1.0 / _dt);
}

void Fitness::reset()
{
    pulNeu.reset();
    pulNeu = make_unique<Matsuoka2Neu>(Matsuoka2Neu::pulsePattern, _dt);
    _terminate = false;
    _crossZero = false;
    _ly = 0.f;


    qD.reset(1.0 / _dt);
    qC.reset(1.0 / _dt);
}

float Fitness::calcFitness(Vector6f param)
{
    //-> Reset
    reset();

    //-> check Params Tu and Tv cannot be zeros.
    const float Tu = param[0];
    const float Tv = param[1];
    if (abs(Tu) < 1e-4 || abs(Tv) < 1e-4)
    {
        //-> The divisor cannot be zero
        _fitness = -10;
        return _fitness;
    }
    //-> Inital fitness;
    float fitness = -1;
    //-> Set Neuron parameters.
    pulNeu->setParam(param);

    //-> Score: maxY
    float maxY = pulNeu->getYAbsMax(MAXCOSTTIME);

    //-> LOG
    _maxY = maxY;

    if (isinf(maxY) || isnan(maxY))
    {
        //-> Not a number
        _fitness = -10.f;
        return _fitness;
    }
    if (abs(maxY) < 1e-2)
    {
        _fitness = -5.f;
        return _fitness;   //-> is Zero
    }
    if (abs(maxY) > 100)
    {
       _fitness = -2.f;
       return _fitness;   //-> too Large MaxY
    } 
    float fOverShoot = gau->calcN(maxY);

    //-> Integral Neuron
    Stat stat = Stat::overtime;
    float t = 0;
    float terminalE = 0.f;  // terminal error
    float cumulatedC = 0.f; // cumulated control
    _crossZero = false;
    while (t < MAXCOSTTIME)
    {
        pulNeu->integrate();
        float y = pulNeu->y();
        // RK::state_type state = pulNeu->getIntegrator().states.back();
        // const float u1 = state[0];
        // const float u2 = state[2];
        // // float y1 = max(0.f, u1);
        // // float y2 = max(0.f, u2);
        // float y1 = u1;
        // float y2 = u2;
        // float y = y2 - y1;

        if (y < 0)
            _crossZero = true;

        cumulatedC += abs(y) / maxY;
        if (isnan(cumulatedC) || isinf(cumulatedC))
            return -10.f;

        float deltaY = abs(y - _ly) / _dt / maxY;
        if (deltaY > MAXSLOPE)
        {
            _fitness = -3.1f;  //-> too large flope.
            return _fitness;
        }
        _ly = y;

        //-> If wrong control direction
        qD.insert(y);
        if (qD.full() && qD.size() > 0)
        {
            if (qD.sign() < 0)
            {
                stat = Stat::wrongDirection;
                break; // Wrong moving direction.
            }
            qD.reset(0);
        }
        //-> If terminate
        const float tValue = 0.01;
        qC.insert(y);
        if (qC.full())
        {
            if (qC.absmax() < tValue || qC.deltaMaxMin() < tValue)
            {
                //-> terminate
                stat = Stat::terminate;
                terminalE = y;
                break;
            }
        }

        t += _dt;
    }
    if (stat == Stat::terminate && _crossZero)
    {
        // float fCostTime = 1.0 - t / MAXCOSTTIME;
        float fCostTime = gauT->calcN(t);
        float fTerminalE = 1.0 / exp(abs(terminalE) / 10.0);
        float nCumulatedC = cumulatedC * _dt / t;
        if (nCumulatedC > 2/M_PI) nCumulatedC = 2/M_PI;
        float fCumulatedC = 0.49*cos(0.5*M_PI*M_PI*nCumulatedC)+0.5;

        fitness = fCostTime * ( w[1] * fTerminalE + w[2] * fCumulatedC + w[3] * fOverShoot );
        // fitness = fCostTime * fTerminalE  * fCumulatedC * fOverShoot;

        _terminate = true;

        //-> Log to Info()
        _costtime = t;
        _cumulatedC = cumulatedC;
        _terminalE = terminalE;
        _fCostTime = fCostTime;
        _fTerminalE = fTerminalE;
        _fCumulatedC = fCumulatedC;
        _fOverShoot = fOverShoot;
        _fitness = fitness;
    }
    else
    {
        fitness = -1.f;
        _fitness = fitness;
    }
    return fitness;
}

void Fitness::test()
{
}

void Fitness::info()
{
    cout << "{\n";
    cout << "\tfitness: " << _fitness << endl;
    cout << "\tcosttime: " << _costtime << endl;
    cout << "\tmaxY: " << _maxY << endl;
    cout << "\tcumulatedC: " << _cumulatedC << endl;
    cout << "\tterminalE: " << _terminalE << endl;
    cout << "\tfCosttime: " << _fCostTime << endl;
    cout << "\tfTerminalE: " << _fTerminalE << endl;
    cout << "\tfCumulatedC: " << _fCumulatedC << endl;
    cout << "\tfOverShoot: " << _fOverShoot << endl;
    cout << "}" << endl;
}
