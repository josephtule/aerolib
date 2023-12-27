#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "external.h"

class Attitude {
  public:
    // construct/destruct
    Attitude(std::initializer_list<f64> EP_in,
             std::initializer_list<f64> EP_dot_in, u32 N);
    Attitude(f64 EP_in[], f64 EP_dot_in[], u32 N);
    Attitude(Vector4d EP, Vector4d EP_dot, u32 N);
    ~Attitude();

    // methods:
    f64 operator()(size_t ind) const;
    void renorm();
    Vector3d EP_dottoOmega(Vector4d quat, Vector4d quat_dot);
    Vector4d OmegatoEP_dot(Vector3d omega, Vector4d quat);

    // Conversions:
    Matrix3d EPtoDCM(Vector4d quat);
    Vector4d DCMtoEP(Matrix3d b_C_n);
    Vector4d EAtoEP(f64 angles[3], u8 seq[3]);
    Vector4d CRPtoEP(Vector3d rho);
    Vector4d MRPtoEP(Vector3d sigma);
    Vector4d PRPtoEP(Vector3d lambda, f64 theta);

    // attributes:
    Vector4d quat;
    Vector4d quat_dot = Vector4d::Zero();
    Vector3d omega = Vector3d::Zero();
    Matrix3d b_C_n = Matrix3d::Identity();
    u32 N;

    // time histories:
    Vector4d *quat_hist = new Vector4d[N + 1];
    Vector3d *omega_hist = new Vector3d[N + 1];
};
#endif
