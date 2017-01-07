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

    const double def_mass = 1e2;

    for (auto& p: data) {
        p.mass = def_mass;
        p.pos = Vec3(rand_uni(rand_eng),rand_uni(rand_eng),0);
        p.vel = Vec3(0.01*rand_uni(rand_eng),0.01*rand_uni(rand_eng),0);
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