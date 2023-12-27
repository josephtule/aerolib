#include "Simulation.h"

void Simulation::propagate(std::vector<Satellite> &satellites, EOMS &eoms,
                           Integrator &integrator, f64 dt, u32 max_steps) {

    auto update_state = [&eoms](double t, VectorXd s) {
        return eoms.dxdt(t, s);
    };

    for (Satellite sat : satellites) {
        for (int i = 0; i < max_steps; i++) {
            VectorXd state(6);
            state << sat.position, sat.velocity;

            VectorXd state_new = integrator.step(update_state, time, state, dt);

            state = state_new;
            sat.position = state(Eigen::seq(0, 2));
            sat.velocity = state(Eigen::seq(3, 5));
        }
    }
}
