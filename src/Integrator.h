#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "external.h"
#include <eigen3/Eigen/src/Core/Matrix.h>

using EOMSFunction = std::function<VectorXd(f64 time, const VectorXd &state)>;

class Integrator {
    enum IntegratorType {
        RK4,
        Euler,
    };

  public:
    Integrator(IntegratorType type)
        : type(type){
              //
          };
    ~Integrator(){};
    VectorXd step(EOMSFunction f, f64 &time, VectorXd &state, f64 dt) {
        switch (type) {
        case Euler:
            return euler_step(f, time, state, dt);
        case RK4:
            return rk4_step(f, time, state, dt);
        // ... other cases ...
        default:
            throw std::runtime_error("Unknown integrator type");
        }
    }
    VectorXd rk4_step(EOMSFunction f, f64 &time, VectorXd &state, f64 dt) {
        VectorXd k1 = f(time, state);
        VectorXd k2 = f(time + dt / 2., state + dt * k1 / 2.);
        VectorXd k3 = f(time + dt / 2., state + dt * k2 / 2.);
        VectorXd k4 = f(time + dt, state + dt * k3);

        VectorXd state_new = state + dt / 6. * (k1 + 2 * k2 + 2 * k3 + k4);
        time += dt;
        return state_new;
    }
    VectorXd euler_step(EOMSFunction f, f64 &time, VectorXd &state, f64 dt) {
        VectorXd state_new = state + dt * f(time, state);
        time += dt;
        return state_new;
    }

    IntegratorType type;
};

#endif
