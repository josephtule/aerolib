#ifndef SATELLITE_H
#define SATELLITE_H

#include "Attitude.h"
#include "external.h"

class Satellite {
  public:
    // constructor/destructor:
    Satellite(Vector3d position, Vector3d velocity, Attitude attitude)
        : position(position), velocity(velocity), attitude(attitude) {
        position_hist.push_back(position);
        velocity_hist.push_back(velocity);
    };
    ~Satellite(){};
    // methods:

    // attributes:
    Vector3d position;
    Vector3d velocity;
    Attitude attitude; // include orientation and angular velocity
    f64 mass;
    Matrix3d I; // inertia matrix
    f64 area;

    // time histories:
    std::vector<Vector3d> position_hist = {};
    std::vector<Vector3d> velocity_hist = {};
};

#endif
