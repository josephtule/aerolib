#include "astrolib.h"
#define PI 3.14159265358979323846

int main() {
    // TODO: set up scenario in matlab and compare to here
    CentralBody earth(5.9722e24, 3.986004418e5, 6378.137); // using km
    Integrator rk4(Integrator::RK4);
    EOMS spherical(earth, EOMS::spherical, EOMS::none, EOMS::combined);
    f64 t0 = 0.;
    f64 dt = 0.125;
    Vector3d pos_0 = {10, 10, 10};
    Vector3d vel_0 = {100, 10, 10};
    Attitude sat1_att({1, 2, 3, 4}, {0, 0, 0, .1});
    Satellite sat1(pos_0, vel_0, sat1_att, "sat1");
    std::vector<Satellite *> sats;
    sats.push_back(&sat1);
    int N = 1000;
    Simulation sim(sats, rk4, spherical, t0, dt, N);
    sim.propagate();
    sim.save_results();
    return 0;
}
