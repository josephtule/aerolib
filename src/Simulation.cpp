#include "Simulation.h"

void Simulation::propagate() {
    EOMS &_eoms = eoms;
    auto update_state = [&_eoms](double t, VectorXd s, Satellite sat) {
        return _eoms.dxdt(t, s, sat);
    };
    VectorXd state;
    VectorXd state_new;

    for (Satellite *sat : satellites) {
        int state_dim = sat->position.size() + sat->velocity.size();
        if (eoms.dof == EOMS::combined) {
            state_dim += sat->attitude.quat.size() + sat->attitude.omega.size();
        }
        state = VectorXd::Zero(state_dim);
        state_new = VectorXd::Zero(state_dim);
        for (int i = 0; i < max_steps; i++) {

            state(Eigen::seq(0, 2)) = sat->position;
            state(Eigen::seq(3, 5)) = sat->velocity;

            if (eoms.dof == EOMS::combined) {
                state(Eigen::seq(6, 9)) = sat->attitude.quat;
                state(Eigen::seq(10, 12)) = sat->attitude.omega;
            }
            integrator.step(update_state, time, state, dt, *sat);

            sat->position = state(Eigen::seq(0, 2));
            sat->velocity = state(Eigen::seq(3, 5));
            sat->position_hist[i + 1] = sat->position;
            sat->velocity_hist[i + 1] = sat->velocity;
            if (eoms.dof == EOMS::combined) {
                sat->attitude.quat = state(Eigen::seq(6, 9));
                sat->attitude.omega = state(Eigen::seq(10, 12));
                sat->attitude.quat_hist[i + 1] = sat->attitude.quat;
                sat->attitude.omega_hist[i + 1] = sat->attitude.omega;
            }
            time_hist[i + 1] = time;
        }
    }
}

void Simulation::save_results() {
    for (Satellite *sat : satellites) {
        std::stringstream ss;
        ss.precision(16);
        std::string sep = ",  ";
        bool check = false;
        if (eoms.dof == EOMS::combined) {
            check = true;
        }
        for (int i = 0; i < max_steps + 1; i++) {
            ss << time_hist[i] << sep;
            append_eigen_vec(ss, sat->position_hist[i], sep);
            ss << sep;
            append_eigen_vec(ss, sat->velocity_hist[i], sep);
            if (check) {
                ss << sep;
                append_eigen_vec(ss, sat->attitude.quat_hist[i], sep);
                ss << sep;
                append_eigen_vec(ss, sat->attitude.omega_hist[i], sep);
            }
            ss << "\n";
        }

        std::ofstream outfile((sat->name) + ".txt");
        outfile << ss.str();
        outfile.close();
    }
}
