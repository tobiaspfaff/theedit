#include "test.hpp"
#include "gravity.hpp"
#include "particle.hpp"
#include "util.hpp"

using namespace std;

void gravity_test() {
    SingleGravityPotential g0(Vec3(0,0,0), 1e0);
    CrossGravityPotential g1;
    const double dt = 1e-2;
    const double out_dt = 1e1;
    const double max_t = 10000;

    ParticleSet pset(100);
    pset.seed_random();

    int frame = 0;
    for (double t=0, lt=0; t < max_t; t += dt, lt += dt) {
        vector<Vec3> f0(pset.size());
        vector<Vec3> f1(pset.size());

        // velocity verlet
        g1.get_force(pset, f0);

        for (int i=0; i<f0.size(); i++) {
            pset[i].pos += dt * pset[i].vel + 0.5 * f0[i] * dt * dt; 
        }

        g1.get_force(pset, f1);

        for (int i=0; i<f0.size(); i++) {
            pset[i].vel += 0.5 * dt * (f0[i] + f1[i]);
        }

        // save
        while (lt > out_dt) {
            cout << frame << " (" << t << ")" << endl;
            lt -= out_dt;
            pset.save(stringf("out/%05d.txt", frame++));
        }
    }
}