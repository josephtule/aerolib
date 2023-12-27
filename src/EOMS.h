#ifndef EOMS_H
#define EOMS_H

#include "CentralBody.h"
#include "Satellite.h"
#include "external.h"

class EOMS {
  public:
    // models:
    enum GravityModel {
        spherical,
        spherical_harmonic,
    };

    enum AerodynamicsModel {
        none,
    };
    EOMS(CentralBody &central_body, GravityModel grav_model,
         AerodynamicsModel aero_model)
        : central_body(central_body), gravity_model(grav_model),
          aerodynamics_model(aero_model){};
    // methods:
    Vector6d dxdt(f64 &time, VectorXd &state) {
        Vector6d dxdt_vec = Vector6d::Zero();
        Vector3d position = state(Eigen::seq(0, 2));

        dxdt_vec(Eigen::seq(0, 2)) = state(Eigen::seq(3, 5));

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
        return dxdt_vec;
    }

    Vector3d spherical_grav(Vector3d &position);
    Vector3d euler_rot(Vector3d &omega, Satellite satellite, Vector3d &torque);
    // attributes:
    std::vector<Satellite> satellites;
    CentralBody central_body;
    GravityModel gravity_model;
    AerodynamicsModel aerodynamics_model;
};

#endif
