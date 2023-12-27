#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "external.h"

class Attitude {
  public:
    // construct/destruct
    Attitude();
    Attitude(std::initializer_list<f64> EP_in,
             std::initializer_list<f64> EP_dot_in);
    Attitude(f64 EP_in[], f64 EP_dot_in[]);
    Attitude(Vector4d EP, Vector4d EP_dot);
    ~Attitude();

    // methods:
    f64 operator()(size_t ind) const;
    void renorm();
    void EP_dottoOmega();
    void OmegatoEP_dot();

    // Conversions:
    void EPtoDCM();
    void DCMtoEP(Matrix3d b_C_n_in);
    void EAtoEP(f64 angles[3], u8 seq[3]);
    void CRPtoEP(Vector3d rho);
    void MRPtoEP(Vector3d sigma);
    void PRPtoEP(Vector3d lambda, f64 theta);
    // attributes:
    Vector4d quat;
    Vector4d quat_dot = Vector4d::Zero();
    Vector3d Omega = Vector3d::Zero();
    Matrix3d b_C_n = Matrix3d::Identity();

    // time histories:
    std::vector<Vector4d> EP_hist;
    std::vector<Vector3d> Omega_hist;
};
#endif
