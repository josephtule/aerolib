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
    Vector4d tempepdot = ep1.OmegatoEP_dot(omega);
    std::cout << tempepdot << std::endl;
    Vector3d tempomega = ep1.EP_dottoOmega(epdot);
    std::cout << tempomega << std::endl;
    return 0;
}
