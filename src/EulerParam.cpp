#include "EulerParam.h"

EulerParam::EulerParam(std::initializer_list<f64> EP_in) {
    std::copy(EP_in.begin(), EP_in.end(), EP.begin());
}

EulerParam::EulerParam(f64 EP_in[]) {
    for (int i = 0; i < 4; i++) {
        EP[i] = EP_in[i];
    }
}

EulerParam::~EulerParam() {}

f64 EulerParam::operator()(size_t ind) const { return EP[ind]; }

void EulerParam::EPtoDCM() {
    b_C_n << 1 - 2 * EP[1] * EP[1] - 2 * EP[2] * EP[2],
        2 * (EP[0] * EP[1] + EP[2] * EP[3]),
        2 * (EP[0] * EP[2] - EP[1] * EP[3]), //
        2 * (EP[0] * EP[1] - EP[2] * EP[3]),
        1 - 2 * EP[0] * EP[0] - 2 * EP[2] * EP[2],
        2 * (EP[1] * EP[2] + EP[0] * EP[3]), //
        2 * (EP[0] * EP[2] + EP[1] * EP[3]),
        2 * (EP[1] * EP[2] - EP[0] * EP[3]),
        1 - 2 * EP[0] * EP[0] - 2 * EP[1] * EP[1];
}

void EulerParam::renorm() {
    f64 norm = 0;
    for (int i = 0; i < 4; i++) {
        norm += EP[i] * EP[i];
    }
    norm = sqrt(norm);

    for (int i = 0; i < 4; i++) {
        EP[i] /= norm;
    }
}

Vector3d EulerParam::EP_dottoOmega(Vector4d EP_dot) {
    Matrix4d mat;
    mat << EP(3), EP(2), -EP(1), -EP(0), -EP(2), EP(3), EP(0), -EP(1), EP(1),
        -EP(0), EP(3), -EP(2), EP(0), EP(1), EP(2), EP(3);

    Vector4d temp = 2 * mat * EP_dot;
    Vector3d omega = temp.head(3);

    return omega;
}

Vector4d EulerParam::OmegatoEP_dot(Vector3d omega) {
    Vector4d EP_dot = Vector4d::Zero();
    Matrix4d mat;
    mat << EP(3), EP(2), -EP(1), -EP(0), -EP(2), EP(3), EP(0), -EP(1), EP(1),
        -EP(0), EP(3), -EP(2), EP(0), EP(1), EP(2), EP(3);
    Vector4d temp = Vector4d::Zero();
    temp.head(3) = omega;
    EP_dot = 1. / 2. * mat.transpose() * temp;

    return EP_dot;
}
