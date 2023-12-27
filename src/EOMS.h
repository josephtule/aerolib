#ifndef EOMS_H
#define EOMS_H

#include "CentralBody.h"
#include "Satellite.h"
#include "external.h"

class EOMS {
  public:
    enum DegreesOfFreedom {
        translational,
        combined,
    };
    // models:
    enum GravityModel {
        spherical,
        spherical_harmonic,
    };

    enum AerodynamicsModel {
        none,
    };
    EOMS(CentralBody &central_body, GravityModel grav_model,
         AerodynamicsModel aero_model, DegreesOfFreedom dof)
        : central_body(central_body), gravity_model(grav_model),
          aerodynamics_model(aero_model), dof(dof){};
    // methods:
    VectorXd dxdt(f64 &time, VectorXd &state, Satellite &sat) {

        // TODO: make state vector size 14 to include translational and angular
        VectorXd dxdt_vec = VectorXd::Zero(state.size());
        Vector3d position = state(Eigen::seq(0, 2));
        Vector3d velocity = state(Eigen::seq(3, 5));
        dxdt_vec(Eigen::seq(0, 2)) = velocity;

        switch (gravity_model) {
        case spherical: {
            dxdt_vec(Eigen::seq(3, 5)) = spherical_grav(position);
            break;
        }
        case spherical_harmonic: {
            // TODO: call spherical harmonic gravity here
            break;
        }
        default: {
            std::cerr << "Error: model unknown" << std::endl;
            break;
        }
        }

        if (dof == combined) {
            Vector4d quat = state(Eigen::seq(6, 9));
            Vector3d omega = state(Eigen::seq(10, 12));
            dxdt_vec(Eigen::seq(6, 9)) =
                sat.attitude.OmegatoEP_dot(omega, quat);

            Vector3d torque = {0, 0, 0};
            dxdt_vec(Eigen::seq(10, 12)) = euler_rot(omega, sat, torque);
        }
        return dxdt_vec;
    }

    Vector3d spherical_grav(Vector3d &position);
    Vector3d euler_rot(Vector3d &omega, Satellite satellite, Vector3d &torque);
    // attributes:
    std::vector<Satellite> satellites;
    CentralBody central_body;
    DegreesOfFreedom dof;
    GravityModel gravity_model;
    AerodynamicsModel aerodynamics_model;
};

#endif
