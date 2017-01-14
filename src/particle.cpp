#include "particle.hpp"
#include <random>
#include <fstream>

using namespace std;

ParticleSet::ParticleSet(int size) {
    data.resize(size);
}

void ParticleSet::seed_random() {
    mt19937 rand_eng(2384);
    uniform_real_distribution<double> rand_uni(-1,1);

    const double def_mass = 1;

    for (auto& p: data) {
        p.mass = def_mass;
        p.pos = Vec3(rand_uni(rand_eng),rand_uni(rand_eng),0);
        p.vel = 0.0*Vec3(0.01*rand_uni(rand_eng),0.01*rand_uni(rand_eng),0);
    }
}

void ParticleSet::seed_unicorn() {
    mt19937 rand_eng(2384);
    uniform_real_distribution<double> rand_uni(-0.01,0.01);

    const double def_mass = 1;

    int H = sqrt(data.size());

    for (int i=0; i<data.size(); i++) {
        double iy = 2.0 * (i % H) / H - 1.0;
        double ix = 2.0 * (i / H) / H - 1.0;
        data[i].mass = def_mass;
        data[i].pos = Vec3(ix + rand_uni(rand_eng), iy + rand_uni(rand_eng), 0);
        //p.vel = 0.0*Vec3(0.01*rand_uni(rand_eng),0.01*rand_uni(rand_eng),0);
    }
}

void ParticleSet::save(const string& filename) {
    ofstream ofs(filename.c_str());
    ofs << size() << endl;
    for (auto& p: data) {
        ofs << p.pos[0] << " " << p.pos[1] << " " << p.pos[2] << endl;
    }
    ofs.close();
}