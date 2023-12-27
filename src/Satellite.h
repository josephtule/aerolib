#ifndef SATELLITE_H
#define SATELLITE_H

#include "Attitude.h"
#include "external.h"

class Satellite {
  public:
    // constructor/destructor:
    Satellite(Vector3d position, Vector3d velocity, Attitude &attitude,
              std::string name, u32 N)
        : position(position), velocity(velocity), attitude(attitude),
          name(name), N(N) {
        position_hist[0] = position;
        velocity_hist[0] = velocity;
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
    std::string name;
    u32 N;
    // time histories:
    Vector3d *position_hist = new Vector3d[N + 1];
    Vector3d *velocity_hist = new Vector3d[N + 1];
};

#endif
