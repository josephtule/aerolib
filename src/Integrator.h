#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "external.h"

class Integrator {
  public:
    Integrator(std::string &integrator_name) {
        if (integrator_name == "euler") {
            // use euler
        } else if (integrator_name == "rk4") {
            // use rk4
        } else {
            // error, integrator not defined
        }
    }
    ~Integrator(){};
};

#endif
