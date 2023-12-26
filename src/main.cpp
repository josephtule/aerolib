#include "Attitude.h"
#include "external.h"
#include <chrono>
#include <iostream>
#define PI 3.14159265358979323846

using namespace std::chrono;
int main() {
    Matrix3d bCn;
    bCn << 0.836516303737808, 0.482962913144534, -0.258819045102521,
        -0.393184592519655, 0.858058344800062, 0.330366089549352,
        0.381636410456325, -0.174592959325177, 0.907673371190369;
    Attitude e;
    e.DCMtoEP(bCn);

    auto start = high_resolution_clock::now();
    e.EPtoDCM();
    auto end = high_resolution_clock::now();
    auto dur = duration_cast<microseconds>(end - start);
    std::cout << "Original C : " << std::endl << e.b_C_n << std::endl;
    std::cout << "Elapsed time: " << dur.count() * 1e-6 << std::endl;
    std::cout << " ----------------------------- " << std::endl;

    f64 ea[] = {PI / 6, PI / 12, 20 * PI / 180.};
    u8 seq[] = {3, 2, 1};
    start = high_resolution_clock::now();
    e.EAtoEP(ea, seq);
    end = high_resolution_clock::now();
    dur = duration_cast<microseconds>(end - start);
    std::cout << "From EA    : " << std::endl << e.b_C_n << std::endl;
    std::cout << "Elapsed time: " << dur.count() * 1e-6 << std::endl;
    std::cout << " ----------------------------- " << std::endl;

    std::cout << "Original EP: " << std::endl << e.EP << std::endl;
    std::cout << " ----------------------------- " << std::endl;

    Vector3d lambda = {0.421854978000572, 0.535051946721835, 0.731954774453526};
    f64 theta = 36.762428160542513 * PI / 180.;
    start = high_resolution_clock::now();
    e.PRPtoEP(lambda, theta);
    end = high_resolution_clock::now();
    dur = duration_cast<microseconds>(end - start);
    std::cout << "From PRP   : " << std::endl << e.EP << std::endl;
    std::cout << "Elapsed time: " << dur.count() * 1e-6 << std::endl;
    std::cout << " ----------------------------- " << std::endl;

    Vector3d rho = {0.140178867781743, 0.177793270216625, 0.243222426902823};
    start = high_resolution_clock::now();
    e.CRPtoEP(rho);
    end = high_resolution_clock::now();
    dur = duration_cast<microseconds>(end - start);
    std::cout << "From CRP   : " << std::endl << e.EP << std::endl;
    std::cout << "Elapsed time: " << dur.count() * 1e-6 << std::endl;
    std::cout << " ----------------------------- " << std::endl;

    Vector3d sigma = {0.068254626885796, 0.086569491632188, 0.118427665034059};
    start = high_resolution_clock::now();
    e.MRPtoEP(sigma);
    end = high_resolution_clock::now();
    dur = duration_cast<microseconds>(end - start);
    std::cout << "From MRP   : " << std::endl << e.EP << std::endl;
    std::cout << "Elapsed time: " << dur.count() * 1e-6 << std::endl;
    std::cout << " ----------------------------- " << std::endl;

    Attitude x({1, 2, 3, 4});
    std::cout << x.EP << std::endl;
    return 0;
}
