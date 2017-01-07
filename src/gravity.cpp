#include "gravity.hpp"
#include "particle.hpp"
#include "util.hpp"

using namespace std;
const double grav_constant = 6.674e-11;

// clamped gravitational force F = -G m_1 m_2 r_12 / |r_12|^3
inline Vec3 gravity_force(const Vec3& x1, const Vec3& x2, double m1, double m2) {
    Vec3 d = x1 - x2;
    double safe_dist = max(1e-3, norm(d));
    return -grav_constant * m1 * m2 / (safe_dist * safe_dist * safe_dist) * d;
}

SingleGravityPotential::SingleGravityPotential(const Vec3& x0, double mass) :
    x0(x0), mass(mass) 
{
}

void SingleGravityPotential::get_force(const ParticleSet& pset, vector<Vec3>& f) {
    for (int i=0; i<f.size(); i++) {
        f[i] += gravity_force(pset[i].pos, x0, pset[i].mass, mass);
    }
}

void CrossGravityPotential::get_force(const ParticleSet& pset, vector<Vec3>& f) {
    for (int i=0; i<f.size(); i++) {
        for (int j=0; j<f.size(); j++) {
            f[i] += gravity_force(pset[i].pos, pset[j].pos, pset[i].mass, pset[j].mass);
        }
    }
}