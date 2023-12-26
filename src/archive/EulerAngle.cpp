#include "EulerAngle.h"

EulerAngle::EulerAngle(std::initializer_list<f64> EA_in,
                       std::initializer_list<u8> seq_in) {
    std::copy(EA_in.begin(), EA_in.end(), EA.begin());
    std::copy(seq_in.begin(), seq_in.end(), seq.begin());
}

EulerAngle::EulerAngle(f64 EA_in[], u8 seq_in[]) {
    for (int i = 0; i < 3; i++) {
        EA[i] = EA_in[i];
        seq[i] = seq_in[i];
    }
}

EulerAngle::~EulerAngle() {}

void EulerAngle::EAtoDCM() {
    Matrix3d r;
    Matrix3d temp = Matrix3d::Identity();
    for (int i = 2; i >= 0; i--) {
        r = EulerAngle::rot(EA[i], seq[i]);
        temp = temp * r;
    }
    b_C_n = temp;
}

f64 EulerAngle::operator()(size_t ind) const { return EA[ind]; }
Matrix3d EulerAngle::rot(f64 angle, u8 axis) {
    Matrix3d out = Matrix3d::Zero();
    f64 ca = cos(angle);
    f64 sa = sin(angle);
    if (axis == 1) {
        out(0, 0) = 1;
        out(1, 1) = ca;
        out(2, 2) = ca;
        out(1, 2) = sa;
        out(2, 1) = -sa;
    } else if (axis == 2) {
        out(0, 0) = ca;
        out(0, 2) = -sa;
        out(1, 1) = 1;
        out(2, 0) = sa;
        out(2, 2) = ca;
    } else if (axis == 3) {
        out(0, 0) = ca;
        out(0, 1) = sa;
        out(1, 0) = -sa;
        out(1, 1) = ca;
        out(2, 2) = 1;
    }

    return out;
}
