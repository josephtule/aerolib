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
    Vector6d dxdt_3dof(f64 time, Vector6d &state) {
        Vector6d dxdt = Vector6d::Zero();

        dxdt(Eigen::seq(0, 2)) = state(Eigen::seq(3, 5));

        switch (gravity_model) {
        case spherical: {
            dxdt(Eigen::seq(3, 5)) = spherical_grav(state(Eigen::seq(0, 2)));
            break;
        }
        case spherical_harmonic: {
            int jdsao = 0;
            break;
        }
        default: {
            std::cerr << "Error: model unknown" << std::endl;
            break;
        }
        }
        return dxdt;
    }

    // position:
    Vector3d spherical_grav(const Vector3d &position) {
        Vector3d acceleration = Vector3d::Zero();
        f64 r = position.norm();
        acceleration = -central_body.mu / r * position;
        return acceleration;
    }
    // attitude:
    Vector3d euler_rot(Vector3d &omega, Satellite satellite, Vector3d &torque) {
        Vector3d ang_accel = Vector3d::Zero();
        ang_accel(0) =
            (-(satellite.I(2, 2) - satellite.I(1, 1)) * omega(1) * omega(2) +
             torque(0)) /
            satellite.I(0, 0);
        ang_accel(1) =
            (-(satellite.I(0, 0) - satellite.I(2, 2)) * omega(2) * omega(0) +
             torque(1)) /
            satellite.I(1, 1);
        ang_accel(2) =
            (-(satellite.I(1, 1) - satellite.I(0, 0)) * omega(0) * omega(1) +
             torque(2)) /
            satellite.I(2, 2);
        return ang_accel;
    }
    // attributes:
    std::vector<Satellite> satellites;
    CentralBody central_body;
    GravityModel gravity_model;
    AerodynamicsModel aerodynamics_model;
};

#endif
