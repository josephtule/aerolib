#ifndef CENTRALBODY_H
#define CENTRALBODY_H

#include "external.h"

class CentralBody {
  public:
    CentralBody(f64 mass, f64 mu, f64 equatorial_radius)
        : mass(mass), mu(mu), equatorial_radius(equatorial_radius) {}
    ~CentralBody(){};
    void legendre_matrix(u32 maxord, u32 maxdeg, std::string filepath) {
        this->maxord;
        this->maxdeg;
    }
    // attributes:
    f64 mass;
    f64 mu;
    f64 equatorial_radius;
    Eigen::MatrixXd C;
    Eigen::MatrixXd S;
    u32 maxord;
    u32 maxdeg;
    f64 rot_rate;
    Matrix3d ecef_C_eci;
};

#endif
