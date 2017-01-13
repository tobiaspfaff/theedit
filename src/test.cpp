#include "test.hpp"
#include "gravity.hpp"
#include "particle.hpp"
#include "util.hpp"

using namespace std;

void gravity_test() {
    const double dt = 1e-3;
    const double out_dt = 1e-2;
    const double max_t = 10;

    RepulsionPotential rp;
    BoxPotential bp(Vec3(-1,-1,-1),Vec3(1,1,1),Vec3(0,-9.81,0));

    ParticleSet pset(200);
    pset.seed_random();

    int frame = 0;
    for (double t=0, lt=0; t < max_t; t += dt, lt += dt) {
        vector<Vec3> f0(pset.size());
        vector<Vec3> f1(pset.size());

        // velocity verlet integration
        bp.get_force(pset, f0);
        rp.get_force(pset, f0);

        for (int i=0; i<f0.size(); i++) {
            pset[i].pos += dt * pset[i].vel + 0.5 * f0[i] * dt * dt; 
        }

        bp.get_force(pset, f1);
        rp.get_force(pset, f1);

        for (int i=0; i<f0.size(); i++) {
            pset[i].vel += 0.5 * dt * (f0[i] + f1[i]);

            // damping
            pset[i].vel *= 0.999;
        }

        // save
        while (lt > out_dt) {
            cout << frame << " (" << t << ")" << endl;
            lt -= out_dt;
            pset.save(stringf("out/%05d.txt", frame++));
        }
    }
}