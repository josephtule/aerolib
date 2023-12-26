#ifndef EULERANGLE_H
#define EULERANGLE_H

#include "external.h"

class EulerAngle {
  public:
    EulerAngle(std::initializer_list<f64> EA_in, std::initializer_list<u8> seq);
    EulerAngle(f64 EA_in[], u8 seq_in[]);
    ~EulerAngle();
    // methods:
    void EAtoDCM();
    void DCMtoEA();
    Matrix3d rot(f64 angle, u8 axis);
    f64 operator()(size_t ind) const;
    // attributes:
    std::array<f64, 3> EA;
    std::array<f64, 3> seq;
    Matrix3d b_C_n = Matrix3d::Identity(); // inertial to body
};

#endif
