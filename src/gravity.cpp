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
            f[i] += -gravity_force(pset[i].pos, pset[j].pos, pset[i].mass, pset[j].mass);
        }
    }
}

void RepulsionPotential::get_force(const ParticleSet& pset, vector<Vec3>& f) {
    double h = 0.1;
    double stiffness = 1e3;
    for (int i=0; i<f.size(); i++) {
        for (int j=0; j<f.size(); j++) {
            double q = norm(pset[i].pos - pset[j].pos) / h;
            double w = 0;
            if (q < 0.5)
                w = 1.0 - 6.0 * square(q) + 6.0 * cubed(q); 
            else if (q < 1.0)
                w = 2.0 * cubed(1.0 - q); 
            f[i] += w * stiffness * normalize(pset[i].pos - pset[j].pos); 
        }
    }
}

void BoxPotential::get_force(const ParticleSet& pset, vector<Vec3>& f) {
    for (int i=0; i<f.size(); i++) {
        // box boundary repulsion
        double stiffness = 1e5;
        for (int k=0; k<3; k++) {
            Vec3 dir;
            dir[k] = 1;
            double lc = max(x0[k] - pset[i].pos[k], 0.0);
            double rc = min(x1[k] - pset[i].pos[k], 0.0);
            f[i] += dir * (lc + rc) * stiffness;
        }
        f[i] += grav;
    }
}