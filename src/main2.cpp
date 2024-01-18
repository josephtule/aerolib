#include "astrolib.h"
#define PI 3.14159265358979323846

int main(int argc, char *argv[]) {
    // TODO: set up scenario in matlab and compare to here
    u32 N;
    if (argc == 1) {
        N = 10000;
    } else {
        N = std::stoi(argv[1]);
    }
    CentralBody earth(5.9722e24, 3.986004418e5, 6378.137); // using km
    Integrator rk4(Integrator::RK4);
    EOMS::DegreesOfFreedom dofs;
    if (argc > 2) {
        if (std::string(argv[2]) == "translational") {
            dofs = EOMS::translational;
        } else if (std::string(argv[2]) == "combined") {
            dofs = EOMS::combined;
        } else {
            std::cerr << "Incorrect DOF flag" << std::endl;
        }
    } else {
        dofs = EOMS::translational;
    }
    EOMS spherical(earth, EOMS::spherical, EOMS::none, dofs);

    if (dofs == EOMS::translational) {
        std::cout << "3-DOF Simulation" << std::endl;
    } else if (dofs == EOMS::combined) {
        std::cout << "6-DOF Simulation" << std::endl;
    }
    f64 t0 = 0.;
    f64 dt = 0.5;
    Vector3d pos_0 = {earth.equatorial_radius + 250., 0., 0.};
    Vector3d vel_0 = {0., sqrt(earth.mu / pos_0.norm()), 0.};
    Attitude sat1_att(Vector4d({0.3510, 0.0554, -0.4127, 0.8387}),
                      Vector3d({.1, .15, .3}), N);
    Satellite sat1(pos_0, vel_0, sat1_att, "sat1", N);
    sat1.I << 125., 0., 0., 0., 125., 0., 0., 0., 100.;
    std::vector<Satellite *> sats;
    sats.push_back(&sat1);
    Simulation sim(sats, rk4, spherical, t0, dt, N);
    sim.propagate();
    if (argc > 3 && std::string(argv[3]) == "save") {
        sim.save_results();
    }

    return 0;
}
