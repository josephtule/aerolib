#ifndef CENTRALBODY_H
#define CENTRALBODY_H

#include "external.h"

class CentralBody {

    // attributes:
    f64 mass;
    f64 mu;
    f64 equatorial_radius;
    Eigen::MatrixXd C;
    Eigen::MatrixXd S;
    u32 maxdeg;
    u32 maxord;
    f64 rot_rate;
    Matrix3d ecef_C_eci;
};

#endif
