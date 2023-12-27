#include "EOMS.h"

VectorXd EOMS::dxdt(f64 &time, VectorXd &state, Satellite &sat) {

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
        dxdt_vec(Eigen::seq(6, 9)) = sat.attitude.OmegatoEP_dot(omega, quat);
        Vector3d torque = {0, 0, 0};
        dxdt_vec(Eigen::seq(10, 12)) = euler_rot(omega, sat, torque);
    }
    return dxdt_vec;
}

// position:
Vector3d EOMS::spherical_grav(Vector3d &position) {
    Vector3d acceleration = Vector3d::Zero();
    f64 r = position.norm();
    acceleration = -central_body.mu / pow(r, 3) * position;
    return acceleration;
}
// attitude:
Vector3d EOMS::euler_rot(Vector3d &omega, Satellite satellite,
                         Vector3d &torque) {
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
