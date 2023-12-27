#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "Satellite.h"
#include "external.h"
#include <eigen3/Eigen/src/Core/Matrix.h>
using EOMSFunction =
    std::function<VectorXd(f64 time, VectorXd state, Satellite sat)>;

class Integrator {
  public:
    enum IntegratorType {
        RK4,
        Euler,
    };
    Integrator(IntegratorType type) : type(type){};
    ~Integrator(){};
    VectorXd step(EOMSFunction f, f64 &time, VectorXd &state, f64 dt,
                  Satellite &sat) {
        switch (type) {
        case Euler:
            return euler_step(f, time, state, dt, sat);
        case RK4:
            return rk4_step(f, time, state, dt, sat);
        // ... other cases ...
        default:
            throw std::runtime_error("Unknown integrator type");
        }
    }
    VectorXd rk4_step(EOMSFunction f, f64 &time, VectorXd &state, f64 dt,
                      Satellite &sat) {
        VectorXd k1 = VectorXd::Zero(state.size());
        VectorXd k2 = VectorXd::Zero(state.size());
        VectorXd k3 = VectorXd::Zero(state.size());
        VectorXd k4 = VectorXd::Zero(state.size());
        k1 = f(time, state, sat);
        k2 = f(time + dt / 2., state + dt * k1 / 2., sat);
        k3 = f(time + dt / 2., state + dt * k2 / 2., sat);
        k4 = f(time + dt, state + dt * k3, sat);

        VectorXd state_new = state + dt / 6. * (k1 + 2 * k2 + 2 * k3 + k4);
        time += dt;
        return state_new;
    }
    VectorXd euler_step(EOMSFunction f, f64 &time, VectorXd &state, f64 dt,
                        Satellite sat) {
        VectorXd state_new = state + dt * f(time, state, sat);
        time += dt;
        return state_new;
    }

    IntegratorType type;
};

#endif
