#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "Satellite.h"
#include "external.h"
#include <eigen3/Eigen/src/Core/Matrix.h>
using EOMSFunction = std::function<VectorXd(
    const f64 &time, const VectorXd &state, const Satellite &sat)>;

class Integrator {
  public:
    enum IntegratorType {
        RK4,
        Euler,
    };
    Integrator(IntegratorType type) : type(type){};
    ~Integrator(){};
    void step(EOMSFunction f, f64 &time, VectorXd &state, f64 dt,
              Satellite &sat) {
        switch (type) {
        case Euler:
            euler_step(f, time, state, dt, sat);
            break;
        case RK4:
            rk4_step(f, time, state, dt, sat);
            break;
        // ... other cases ...
        default:
            std::cerr << "Unknown integrator type" << std::endl;
        }
    }
    void rk4_step(EOMSFunction f, f64 &time, VectorXd &state, f64 &dt,
                  Satellite &sat) {

        VectorXd k1 = f(time, state, sat);
        VectorXd k2 = f(time + dt / 2., state + dt * k1 / 2., sat);
        VectorXd k3 = f(time + dt / 2., state + dt * k2 / 2., sat);
        VectorXd k4 = f(time + dt, state + dt * k3, sat);

        state += dt / 6. * (k1 + 2 * k2 + 2 * k3 + k4);
        time += dt;
    }
    void euler_step(EOMSFunction f, f64 &time, VectorXd &state, f64 dt,
                    Satellite sat) {
        state += dt * f(time, state, sat);
        time += dt;
    }

    IntegratorType type;
};

#endif
