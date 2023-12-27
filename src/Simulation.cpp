#include "Simulation.h"

void Simulation::propagate() {
    EOMS &_eoms = eoms;
    auto update_state = [&_eoms](double t, VectorXd s) {
        return _eoms.dxdt(t, s);
    };

    for (Satellite *sat : satellites) {
        for (int i = 0; i < max_steps; i++) {
            VectorXd state =
                VectorXd::Zero(sat->position.size() + sat->velocity.size());
            state(Eigen::seq(0, 2)) = sat->position;
            state(Eigen::seq(3, 5)) = sat->velocity;

            VectorXd state_new = integrator.step(update_state, time, state, dt);

            state = state_new;
            sat->position = state(Eigen::seq(0, 2));
            sat->velocity = state(Eigen::seq(3, 5));
            sat->position_hist.push_back(sat->position);
            sat->velocity_hist.push_back(sat->velocity);
            time_hist.push_back(time);
        }
    }
}
