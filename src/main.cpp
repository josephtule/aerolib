#include "Attitude.h"
#include "external.h"
#include <iostream>
int main() {

    EulerParam ep1({42, 1, 4, 31});
    ep1.renorm();
    std::cout << "[" << ep1.EP << "]" << std::endl;
    ep1.EPtoDCM();
    std::cout << "[" << ep1.b_C_n << "]" << std::endl;
    Vector4d epdot = {1, 3, 2, 4};
    Vector3d omega = {1, 2, 3};

    return 0;
}
