#include "Attitude.h"

Matrix3d rot(f64 angle, u8 axis);

Attitude::Attitude() {}

Attitude::Attitude(std::initializer_list<f64> EP_in,
                   std::initializer_list<f64> EP_dot_in) {
    std::copy(EP_in.begin(), EP_in.end(), quat.data());
    std::copy(EP_dot_in.begin(), EP_dot_in.end(), quat_dot.data());
    Attitude::renorm();
}

Attitude::Attitude(f64 EP_in[], f64 EP_dot_in[]) {
    for (int i = 0; i < 4; i++) {
        quat[i] = EP_in[i];
        quat_dot[i] = EP_dot_in[i];
    }
    Attitude::renorm();
}

Attitude::Attitude(Vector4d EP, Vector4d EP_dot) : quat(EP), quat_dot(EP_dot) {
    Attitude::renorm();
}

Attitude::~Attitude() {}

f64 Attitude::operator()(size_t ind) const { return quat[ind]; }

void Attitude::EPtoDCM() {
    b_C_n << 1 - 2 * quat[1] * quat[1] - 2 * quat[2] * quat[2],
        2 * (quat[0] * quat[1] + quat[2] * quat[3]),
        2 * (quat[0] * quat[2] - quat[1] * quat[3]), //
        2 * (quat[0] * quat[1] - quat[2] * quat[3]),
        1 - 2 * quat[0] * quat[0] - 2 * quat[2] * quat[2],
        2 * (quat[1] * quat[2] + quat[0] * quat[3]), //
        2 * (quat[0] * quat[2] + quat[1] * quat[3]),
        2 * (quat[1] * quat[2] - quat[0] * quat[3]),
        1 - 2 * quat[0] * quat[0] - 2 * quat[1] * quat[1];
}

void Attitude::renorm() { quat /= quat.norm(); }

void Attitude::EP_dottoOmega() { // KDE
    Matrix4d mat;
    mat << quat(3), quat(2), -quat(1), -quat(0), -quat(2), quat(3), quat(0),
        -quat(1), quat(1), -quat(0), quat(3), -quat(2), quat(0), quat(1),
        quat(2), quat(3);

    Vector4d temp = 2 * mat * quat_dot;
    Vector3d Omega = temp.head(3);
}

void Attitude::OmegatoEP_dot() { // KDE
    Vector4d EP_dot = Vector4d::Zero();
    Matrix4d mat;
    mat << quat(3), quat(2), -quat(1), -quat(0), -quat(2), quat(3), quat(0),
        -quat(1), quat(1), -quat(0), quat(3), -quat(2), quat(0), quat(1),
        quat(2), quat(3);
    Vector4d temp = Vector4d::Zero();
    temp.head(3) = Omega;
    EP_dot = 1. / 2. * mat.transpose() * temp;
}

void Attitude::DCMtoEP(Matrix3d C) {
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

    double es[6] = {(C(0, 1) + C(1, 0)) / 4.0, (C(1, 2) + C(2, 1)) / 4.0,
                    (C(2, 0) + C(0, 2)) / 4.0, (C(1, 2) - C(2, 1)) / 4.0,
                    (C(2, 0) - C(0, 2)) / 4.0, (C(0, 1) - C(1, 0)) / 4.0};

    switch (ind) {
    case 0:
        quat << e_test(0) * e_test(0), es[0], es[2], es[3];
        quat /= e_test(0);
        break;
    case 1:
        quat << es[0], e_test(1) * e_test(1), es[1], es[4];
        quat /= e_test(1);
        break;
    case 2:
        quat << es[2], es[1], e_test(2) * e_test(2), es[5];
        quat /= e_test(2);
        break;
    case 3:
        quat << es[3], es[4], es[5], e_test(3) * e_test(3);
        quat /= e_test(3);
        break;
    default:
        // Handle unexpected case
        std::cerr << "Unexpected case encountered!" << std::endl;
    }
}

void Attitude::EAtoEP(f64 angles[3], u8 seq[3]) {
    b_C_n = rot(angles[2], seq[2]) * rot(angles[1], seq[1]) *
            rot(angles[0], seq[0]);
    Attitude::DCMtoEP(b_C_n);
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

void Attitude::PRPtoEP(Vector3d lambda, f64 theta) {
    lambda = lambda / lambda.norm();
    quat.head(3) = lambda * sin(theta / 2.);
    quat[3] = cos(theta / 2.);
}

void Attitude::CRPtoEP(Vector3d rho) {
    f64 denom = sqrt(1. + rho.dot(rho));
    quat.head(3) = rho / denom;
    quat[3] = 1. / denom;
}

void Attitude::MRPtoEP(Vector3d sigma) {
    f64 denom = 1. + sigma.dot(sigma);
    quat.head(3) = 2. * sigma / denom;
    quat[3] = (1. - sigma.dot(sigma)) / denom;
}
