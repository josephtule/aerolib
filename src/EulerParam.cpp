#include "EulerParam.h"
Matrix3d rot(f64 angle, u8 axis);

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

void EulerParam::EP_dottoOmega() {
    Matrix4d mat;
    mat << EP(3), EP(2), -EP(1), -EP(0), -EP(2), EP(3), EP(0), -EP(1), EP(1),
        -EP(0), EP(3), -EP(2), EP(0), EP(1), EP(2), EP(3);

    Vector4d temp = 2 * mat * EP_dot;
    Vector3d Omega = temp.head(3);
}

void EulerParam::OmegatoEP_dot() {
    Vector4d EP_dot = Vector4d::Zero();
    Matrix4d mat;
    mat << EP(3), EP(2), -EP(1), -EP(0), -EP(2), EP(3), EP(0), -EP(1), EP(1),
        -EP(0), EP(3), -EP(2), EP(0), EP(1), EP(2), EP(3);
    Vector4d temp = Vector4d::Zero();
    temp.head(3) = Omega;
    EP_dot = 1. / 2. * mat.transpose() * temp;
}

void EulerParam::DCMtoEP(Matrix3d C) {
    // using Shepperds Method

    Eigen::Vector4d e_test = Eigen::Vector4d::Zero();
    double trace_C = C.trace();

    for (int i = 0; i < 4; ++i) {
        if (i < 3) {
            e_test(i) = std::sqrt(0.25 * (1 + 2 * C(i, i) - trace_C));
        } else {
            e_test(i) = std::sqrt(0.25 * (1 + trace_C));
        }
    }

    // Find the index of the maximum element in e_test
    Eigen::Index ind;
    e_test.maxCoeff(&ind);

    Eigen::Vector4d EP;
    double es[6] = {(C(0, 1) + C(1, 0)) / 4.0, (C(1, 2) + C(2, 1)) / 4.0,
                    (C(2, 0) + C(0, 2)) / 4.0, (C(1, 2) - C(2, 1)) / 4.0,
                    (C(2, 0) - C(0, 2)) / 4.0, (C(0, 1) - C(1, 0)) / 4.0};

    switch (ind) {
    case 0:
        EP << e_test(0) * e_test(0), es[0], es[2], es[3];
        EP /= e_test(0);
        break;
    case 1:
        EP << es[0], e_test(1) * e_test(1), es[1], es[4];
        EP /= e_test(1);
        break;
    case 2:
        EP << es[2], es[1], e_test(2) * e_test(2), es[5];
        EP /= e_test(2);
        break;
    case 3:
        EP << es[3], es[4], es[5], e_test(3) * e_test(3);
        EP /= e_test(3);
        break;
    default:
        // Handle unexpected case
        std::cerr << "Unexpected case encountered!" << std::endl;
    }
}

void EulerParam::EAtoEP(f64 angles[3], u8 seq[3]) {
    for (int i = 2; i >= 0; i--) {
        Matrix3d r = rot(angles[i], seq[i]);
        b_C_n = b_C_n * r;
    }
    EulerParam::DCMtoEP(b_C_n);
}

Matrix3d rot(f64 angle, u8 axis) {
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
