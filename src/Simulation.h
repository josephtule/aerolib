#ifndef SIMULATION_H
#define SIMULATION_H

#include "EOMS.h"
#include "Integrator.h"
#include "Satellite.h"
#include "external.h"
#include <iomanip>

class Simulation {
  public:
    // constructor/destructor:
    Simulation(std::vector<Satellite *> &satellites, Integrator &integrator,
               EOMS eoms, f64 time, f64 dt, u32 max_steps)
        : satellites(satellites), integrator(integrator), time(time),
          eoms(eoms), dt(dt), max_steps(max_steps) {
        time_hist[0] = time;
    };
    // methods:
    void add_body(Satellite *new_satellite) {
        satellites.push_back(new_satellite);
    };

    static void out_eigen_vec(std::ofstream &outfile, VectorXd vec,
                              std::string separator) {
        for (int k = 0; k < vec.size(); k++) {
            outfile << vec[k];
            if (k < vec.size() - 1) {
                outfile << separator;
            }
        }
    }
    void save_results();
    void propagate();

    // attributes:
    Integrator integrator;
    EOMS eoms;

    f64 time;
    f64 dt;
    u32 max_steps;
    f64 *time_hist = new f64[max_steps + 1];
    std::vector<Satellite *> satellites; // vector of satellites
};

#endif
