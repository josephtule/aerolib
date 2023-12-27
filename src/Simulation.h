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
               EOMS eoms, f64 time_0, f64 dt, u32 max_steps)
        : satellites(satellites), integrator(integrator), time(time_0),
          eoms(eoms), dt(dt), max_steps(max_steps) {
        time_hist.push_back(time);
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
    void save_results() {
        for (Satellite *sat : satellites) {
            std::ofstream outfile;
            outfile.open((sat->name) + ".txt");
            outfile << std::setprecision(16);
            for (int i = 0; i < max_steps; i++) {
                outfile << time_hist[i] << ",  ";
                out_eigen_vec(outfile, sat->position_hist[i], ",  ");
                outfile << ",  ";
                out_eigen_vec(outfile, sat->velocity_hist[i], ",  ");

                if (eoms.dof == EOMS::combined) {
                    outfile << ",  ";
                    out_eigen_vec(outfile, sat->attitude.quat_hist[i], ",  ");
                    outfile << ",  ";
                    out_eigen_vec(outfile, sat->attitude.quat_dot_hist[i],
                                  ",  ");
                }
                outfile << std::endl;
            }
            outfile.close();
        }
    };
    void propagate();

    // attributes:
    Integrator integrator;
    EOMS eoms;

    f64 time;
    f64 dt;
    u32 max_steps;
    std::vector<f64> time_hist = {};
    std::vector<Satellite *> satellites; // vector of satellites
};

#endif
