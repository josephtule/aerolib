#ifndef SIMULATION_H
#define SIMULATION_H

#include "EOMS.h"
#include "Integrator.h"
#include "Satellite.h"
#include "external.h"

class Simulation {
  public:
    // constructor/destructor:
    Simulation(std::vector<Satellite *> &satellites, Integrator &integrator,
               EOMS eoms, f64 time_0)
        : satellites(satellites), integrator(integrator), time(time_0),
          eoms(eoms) {
        time_hist.push_back(time);
    };
    // methods:
    // void add_body(Satellite new_satellite) {
    //     satellites.push_back(new_satellite);
    // };
    void propagate(/* std::vector<Satellite> &satellites, EOMS &eoms,
                   Integrator &integrator,*/
                   f64 dt, u32 max_steps);
    // attributes:
    Integrator integrator;
    EOMS eoms;

    f64 time;
    std::vector<f64> time_hist = {};
    std::vector<Satellite *> satellites; // vector of satellites
};

#endif
