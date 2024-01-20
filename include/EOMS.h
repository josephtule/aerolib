#ifndef EOMS_H
#define EOMS_H

#include "CentralBody.h"
#include "Satellite.h"
#include "external.h"

class EOMS {
  public:
    enum DegreesOfFreedom {
        translational,
        combined,
    };
    // models:
    enum GravityModel {
        spherical,
        spherical_harmonic,
    };

    enum AerodynamicsModel {
        none,
    };
    EOMS(CentralBody &central_body, GravityModel grav_model,
         AerodynamicsModel aero_model, DegreesOfFreedom dof)
        : central_body(central_body), gravity_model(grav_model),
          aerodynamics_model(aero_model), dof(dof){};
    // methods:
    VectorXd dxdt(f64 &time, VectorXd &state, Satellite &sat);

    Vector3d spherical_grav(Vector3d &position);
    Vector3d euler_rot(Vector3d &omega, Satellite satellite, Vector3d &torque);
    // attributes:
    std::vector<Satellite> satellites;
    CentralBody central_body;
    DegreesOfFreedom dof;
    GravityModel gravity_model;
    AerodynamicsModel aerodynamics_model;
};

#endif
