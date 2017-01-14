#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include "vectors.hpp"
#include <vector>

struct ParticleData {
    double mass = 1;
    Vec3 pos;
    Vec3 vel;
    Vec3 color = Vec3(0,0,0);
};

class ParticleSet {
public:
    ParticleSet(int size);

    inline int size() const { return data.size(); }
    inline ParticleData& operator[](int idx) { return data[idx]; }
    inline const ParticleData& operator[](int idx) const { return data[idx]; }
    inline std::vector<ParticleData>& ptr() { return data; }

    void seed_random();
    void seed_unicorn();
    void save(const std::string& filename);

private:
    std::vector<ParticleData> data;
};

#endif