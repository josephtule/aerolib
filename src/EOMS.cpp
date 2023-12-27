#include "EOMS.h"

// position:
Vector3d EOMS::spherical_grav(Vector3d &position) {
    Vector3d acceleration = Vector3d::Zero();
    f64 r = position.norm();
    acceleration = -central_body.mu / r * position;
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
