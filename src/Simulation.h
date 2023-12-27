#ifndef SIMULATION_H
#define SIMULATION_H

#include "EOMS.h"
#include "Integrator.h"
#include "Satellite.h"
#include "external.h"

class Simulation {
  public:
    // constructor/destructor:
    Simulation(std::vector<Satellite> &satellites, Integrator &integrator)
        : satellites(satellites), integrator(integrator){};
    // methods:
    void add_body(Satellite new_satellite) {
        satellites.push_back(new_satellite);
    };
    void propagate(std::vector<Satellite> &satellites, EOMS &eoms,
                   Integrator &integrator);
    // attributes:
    Integrator integrator;
    EOMS eoms;

    std::vector<Satellite> satellites; // vector of satellites
};

#endif
