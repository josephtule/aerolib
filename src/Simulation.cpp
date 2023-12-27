#include "Simulation.h"

void Simulation::propagate() {
    EOMS &_eoms = eoms;
    auto update_state = [&_eoms](double t, VectorXd s, Satellite sat) {
        return _eoms.dxdt(t, s, sat);
    };

    for (Satellite *sat : satellites) {
        for (int i = 0; i < max_steps; i++) {
            int state_dim = sat->position.size() + sat->velocity.size();
            if (eoms.dof == EOMS::combined) {
                state_dim +=
                    sat->attitude.quat.size() + sat->attitude.quat.size();
            }
            VectorXd state = VectorXd::Zero(state_dim);
            state(Eigen::seq(0, 2)) = sat->position;
            state(Eigen::seq(3, 5)) = sat->velocity;
            if (eoms.dof == EOMS::combined) {
                state(Eigen::seq(6, 9)) = sat->attitude.quat;
                state(Eigen::seq(10, 13)) = sat->attitude.quat_dot;
            }
            VectorXd state_new =
                integrator.step(update_state, time, state, dt, *sat);

            state = state_new;
            // TODO: push attitude too

            sat->position = state(Eigen::seq(0, 2));
            sat->velocity = state(Eigen::seq(3, 5));
            sat->position_hist.push_back(sat->position);
            sat->velocity_hist.push_back(sat->velocity);
            if (eoms.dof == EOMS::combined) {
                sat->attitude.quat = state(Eigen::seq(6, 9));
                sat->attitude.quat_dot = state(Eigen::seq(10, 13));
                sat->attitude.quat_hist.push_back(sat->attitude.quat);
                sat->attitude.quat_dot_hist.push_back(sat->attitude.quat_dot);
            }
            time_hist.push_back(time);
        }
    }
}
