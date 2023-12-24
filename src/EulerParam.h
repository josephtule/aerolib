#ifndef QUATERNION_H
#define QUATERNION_H

#include "external.h"

class EulerParam {
  public:
    // construct/destruct
    EulerParam(std::initializer_list<f64> EP_in);
    EulerParam(f64 EP_in[]);
    ~EulerParam();

    // methods:
    void EPtoDCM();
    f64 operator()(size_t ind) const;
    void renorm();
    Vector3d EP_dottoOmega(Vector4d EP_dot);
    Vector4d OmegatoEP_dot(Vector3d omega);
    // attributes:
    Vector4d EP;
    Matrix3d b_C_n = Matrix3d::Identity();
};
#endif
