#include "Simulation.h"

void Simulation::propagate(std::vector<Satellite> &satellites, EOMS &eoms,
                           Integrator &integrator, f64 dt, u32 max_steps) {
    auto stateUpdateFunction = [&eoms](double t, Eigen::VectorXd &s) {
        return eoms.dxdt(t, s); // or dxdt_6dof
    };
    for (Satellite sat : satellites) {
        for (int i = 0; i < max_steps; i++) {
            VectorXd state(6);
            state << sat.position, sat.velocity;
            // integrator.step(std::bind(&EOMS::dxdt_3dof, &eoms,
            //                           std::placeholders::_1,
            //                           std::placeholders::_2),
            //                 time, state, dt);
            VectorXd state_new = integrator.step(
                [&eoms](f64 t, const Eigen::VectorXd &s) {
                    return eoms.dxdt(t, s);
                },
                time, state, dt);

            state = state_new;
            sat.position = state(Eigen::seq(0, 2));
            sat.velocity = state(Eigen::seq(3, 5));
        }
    }
}
