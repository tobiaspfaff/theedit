#include "test.hpp"
#include "gravity.hpp"
#include "particle.hpp"
#include "util.hpp"
#include "ceres/ceres.h"
#include "glog/logging.h"

using namespace std;
using namespace ceres;

struct ForwardEuler {
    auto static create() {
        return new AutoDiffCostFunction<ForwardEuler, 2, 2, 2, 2>(new ForwardEuler());
    }

    template <typename T>
    bool operator()(const T* const p0, const T* const p1, const T* const v0, T* res) const {
        const double dt = 0.1;

        res[0] = (p1[0] - p0[0]) - dt * (v0[0]);
        res[1] = (p1[1] - p0[1]) - dt * (v0[1]);
        return true;
    }
};

struct SimpleGravity {
    auto static create() {
        return new AutoDiffCostFunction<SimpleGravity, 2, 2, 2>(new SimpleGravity());
    }

    template <typename T>
    bool operator()(const T* const v0, const T* const v1, T* res) const {
        const double dt = 0.1;
        const double g = -9.81;

        res[0] = (v1[0] - v0[0]) - dt * T(0);
        res[1] = (v1[1] - v0[1]) - dt * T(g);
        return true;
    }
};

struct DirichletBoundary {
    auto static create(double x, double y) {
        return new AutoDiffCostFunction<DirichletBoundary, 2, 2>(new DirichletBoundary(x,y));
    }
    DirichletBoundary(double x, double y) : x0(x), y0(y) {}

    template <typename T>
    bool operator()(const T* const p, T* res) const {
        res[0] = p[0] - (T)(x0);
        res[1] = p[1] - (T)(y0);
        return true;
    }

    double x0,y0;
};

// TODO: This doesn't actually work :) Wall constraints in variational mode are funny.
// Let's figure out how to do this right. 
class WallReflect : public SizedCostFunction<1, 2> {
    public:
    auto static create() {
        return new WallReflect();
    }
    virtual ~WallReflect() {}
    bool Evaluate(double const* const* parameters,
                        double* res,
                        double** J) const override {
        const double* p = parameters[0];
        
        res[0] = 0;
        if (p[1] < 0) {
            res[0] = p[1];
        }

        if (J && J[0]) {
            double* dp = J[0];
            // J[parameter_set][#res * pars + p]
            dp[0] = 0;
            dp[1] = 0;
            if (p[1] < 0) {
                dp[1] = 1;
            }
        }
        return true;
    }
};


void gravity_test() {
    const int timesteps = 100;
    google::InitGoogleLogging("simedit");
    vector<double> positions(timesteps*2);
    vector<double> velocities(timesteps*2);

    Problem problem;

    problem.AddResidualBlock(DirichletBoundary::create(0,10), nullptr, &positions[0]);
    problem.AddResidualBlock(DirichletBoundary::create(0,0), nullptr, &velocities[0]);

    for (int i=0; i<timesteps-1; i++) {
        int offset = i*2;

        problem.AddResidualBlock(ForwardEuler::create(), nullptr, &positions[offset], &positions[offset+2], &velocities[offset]);
        problem.AddResidualBlock(SimpleGravity::create(), nullptr, &velocities[offset], &velocities[offset+2]);
        //problem.AddResidualBlock(WallReflect::create(), nullptr, &positions[offset]);
    }
    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    
    for (int i=0; i<timesteps; i++)
        cout << "p:" << positions[i*2] << "," << positions[i*2+1] << " v:" << velocities[i*2] << "," << velocities[i*2+1] << endl;

    exit(1);
/*
    const double dt = 1e-3;
    const double out_dt = 1e-2;
    const double max_t = 10;

    RepulsionPotential rp;
    BoxPotential bp(Vec3(-1,-1,-1),Vec3(1,1,1),Vec3(0,-9.81,0));

    ParticleSet pset(100);
    pset.seed_random();
    pset.seed_unicorn();

    vector< vector<Vec3> > state;

    int frame = 0;
    for (double t=0, lt=0; t < max_t; t += dt, lt += dt) {
        vector<Vec3> f0(pset.size());
        vector<Vec3> f1(pset.size());
        const double damp = 0.998;

        vector<Vec3> x(pset.size());
        for (int i=0; i<f0.size(); i++) {
            x[i] = pset[i].pos;
        }
        state.push_back(x);

        bp.get_force(pset, f0);
        rp.get_force(pset, f0);

        if (1) {
            // velocity verlet integration
            for (int i=0; i<f0.size(); i++) {
                pset[i].pos += dt * pset[i].vel + 0.5 * f0[i] / pset[i].mass * dt * dt; 
            }

            bp.get_force(pset, f1);
            rp.get_force(pset, f1);

            for (int i=0; i<f0.size(); i++) {
                pset[i].vel += 0.5 * dt * (f0[i] + f1[i]) / pset[i].mass;

                // damping
                pset[i].vel *= damp;
            }
        } else {
            // fwd euler
            for (int i=0; i<f0.size(); i++) {
                pset[i].pos += dt * pset[i].vel;
                pset[i].vel += dt * f0[i] / pset[i].mass;
                pset[i].vel *= damp;
            }
        }
    }

*/
    /*    // save
        while (lt > out_dt) {
            cout << frame << " (" << t << ")" << endl;
            lt -= out_dt;
            pset.save(stringf("out/%05d.txt", frame++));
        }
    }*/
}