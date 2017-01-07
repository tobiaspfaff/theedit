#ifndef GRAVITY_HPP
#define GRAVITY_HPP

#include "potential.hpp"
#include "vectors.hpp"
#include <vector>

class ParticleSet;

class SingleGravityPotential : public Potential {
public:
    SingleGravityPotential(const Vec3& x0, double mass);

    void get_force(const ParticleSet& pset, std::vector<Vec3>& forces);

private:
    Vec3 x0;
    double mass;
};

class CrossGravityPotential : public Potential {
public:
    CrossGravityPotential() {}

    void get_force(const ParticleSet& pset, std::vector<Vec3>& forces);

private:
};

#endif