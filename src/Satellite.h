#ifndef SATELLITE_H
#define SATELLITE_H

#include "Attitude.h"
#include "external.h"

class Satellite {

    // constructor/destructor:
    Satellite(Vector3d position, Vector3d velocity, Attitude attitude) {
        this->position = position;
        this->velocity = velocity;
        this->attitude = attitude;
    }
    ~Satellite(){};
    // methods:

    // attributes:
    Vector3d position;
    Vector3d velocity;
    Attitude attitude; // include orientation and angular velocity
    f64 mass;
    Matrix3d I; // inertia matrix
    f64 area;
};

#endif
