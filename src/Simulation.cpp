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
                state(Eigen::seq(10, 12)) = sat->attitude.omega;
            }
            VectorXd state_new =
                integrator.step(update_state, time, state, dt, *sat);

            state = state_new;
            // TODO: push attitude too

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
        std::ofstream outfile;
        outfile.open((sat->name) + ".txt");
        outfile << std::setprecision(16);
        for (int i = 0; i < max_steps + 1; i++) {
            outfile << time_hist[i] << ",  ";
            out_eigen_vec(outfile, sat->position_hist[i], ",  ");
            outfile << ",  ";
            out_eigen_vec(outfile, sat->velocity_hist[i], ",  ");
            if (eoms.dof == EOMS::combined) {
                outfile << ",  ";
                out_eigen_vec(outfile, sat->attitude.quat_hist[i], ",  ");
                outfile << ",  ";
                out_eigen_vec(outfile, sat->attitude.omega_hist[i], ",  ");
            }
            outfile << std::endl;
        }
        outfile.close();
    }
};
