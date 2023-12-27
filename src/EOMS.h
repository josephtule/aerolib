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
    // methods:
    Vector6d dxdt(f64 &time, Vector12d &state) {
        Vector6d dxdt = Vector6d::Zero();
        Vector3d position = state(Eigen::seq(0, 2));

        dxdt(Eigen::seq(0, 2)) = state(Eigen::seq(3, 5));

        switch (gravity_model) {
        case spherical: {
            dxdt(Eigen::seq(3, 5)) = spherical_grav(position);
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
        return dxdt;
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
