#include "test.hpp"
#include "gravity.hpp"
#include "particle.hpp"
#include "util.hpp"
#include "ceres/ceres.h"
#include "glog/logging.h"
#include <fstream>

using namespace std;
using namespace ceres;

struct ForwardEuler {
    auto static create() {
        return new AutoDiffCostFunction<ForwardEuler, 2, 2, 2, 2>(new ForwardEuler());
    }

    template <typename T>
    bool operator()(const T* const p0, const T* const p1, const T* const v0, T* res) const {
        const double dt = 0.1;
        const double alpha = 100;

        res[0] = alpha*((p1[0] - p0[0]) - dt * (v0[0]));
        res[1] = alpha*((p1[1] - p0[1]) - dt * (v0[1]));
        return true;
    }
};

class SimpleGravity : public SizedCostFunction<2, 2, 2, 2> {
public:
    auto static create() {
        return new SimpleGravity();
    }

    virtual ~SimpleGravity() {}
    bool Evaluate(double const* const* parameters,
                        double* res,
                        double** J) const override {
        const double* p0 = parameters[0];
        const double* v0 = parameters[1];
        const double* v1 = parameters[2];

        const double dt = 0.1;
        const double g = -9.81;

        double stiff = 100;
        double rep = 0;
        if (p0[1] < 0)
            rep = stiff * (-p0[1]);

        res[0] = (v1[0] - v0[0]) - dt * (0);
        res[1] = (v1[1] - v0[1]) - dt * (g + rep);

        if (J) {
            if (J[0]) {
                J[0][0*2+0] = 0; // dres[0]/dp0[0] 
                J[0][0*2+1] = 0; // dres[0]/dp0[1]
                J[0][1*2+0] = 0; // dres[1]/dp0[0]
                J[0][1*2+1] = 0; // dres[1]/dp0[1]
                if (p0[1] < 0)
                    J[0][1*2+1] = -dt * stiff * (-1); // dres[1]/dp0[1]
            }
            if (J[1]) {
                J[1][0*2+0] = -1; // dres[0]/dv0[0] 
                J[1][0*2+1] = 0; // dres[0]/dv0[1]
                J[1][1*2+0] = 0; // dres[1]/dv0[0]
                J[1][1*2+1] = -1; // dres[1]/dv0[1]
            }
            if (J[2]) {
                J[2][0*2+0] = 1; // dres[0]/dv1[0] 
                J[2][0*2+1] = 0; // dres[0]/dv1[1]
                J[2][1*2+0] = 0; // dres[1]/dv1[0]
                J[2][1*2+1] = 1; // dres[1]/dv1[1]
            }
        }


        return true;
    }
};

struct SimpleGravityFunctor {
    auto static create() {
        return new AutoDiffCostFunction<SimpleGravityFunctor, 2, 2, 2>(new SimpleGravityFunctor());
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
        double stiff = 10;
        res[0] = stiff* (p[0] - (T)(x0));
        res[1] = stiff* (p[1] - (T)(y0));
        return true;
    }

    double x0,y0;
};
/*
class WallReflect : public SizedCostFunction<2, 2, 2, 2, 2> {
    public:
    auto static create() {
        return new WallReflect();
    }
    virtual ~WallReflect() {}
    bool Evaluate(double const* const* parameters,
                        double* res,
                        double** J) const override {
        const double* p0 = parameters[0];
        const double* p1 = parameters[1];
        const double* v0 = parameters[2];
        const double* v1 = parameters[3];
        
        res[0] = 0;
        res[1] = 0;
        if (p0[1] > 0 && p1[1] < 0) {
            res[0] = 0;
        }

        if (J && J[0]) {
            double* dp = J[0];
            // J[parameter_set][#res * pars + p]
            dp[0] = 0;
            dp[1] = 0;
            if (p0[1] < 0) {
                dp[1] = 1;
            }
        }
        return true;
    }
};
*/

// katana : x/ok
// tanto : ok
// ennead: x
// gridiron: ok
// nodachi: ok

void gravity_test() {
    const int timesteps = 2500;
    google::InitGoogleLogging("simedit");
    vector<double> positions(timesteps*2);
    vector<double> velocities(timesteps*2);

    Problem problem;

    problem.AddResidualBlock(DirichletBoundary::create(0,10), nullptr, &positions[0]);
    problem.AddResidualBlock(DirichletBoundary::create(0,0), nullptr, &velocities[0]);

    //problem.AddResidualBlock(DirichletBoundary::create(0,0), nullptr, &positions[(timesteps-1)*2]);
    for (int i=0; i<positions.size(); i++)
        positions[i] = 1;
/*
    const double dt = 0.1;
    const double g = -9.81;

    double stiff = 100;
    double px = 0, py = 10, vx = 0, vy = 0;
    for (int i=0; i<timesteps; i++) {
        int offset = i*2;
        positions[offset] = px;
        positions[offset+1] = py;
        velocities[offset] = vx;
        velocities[offset+1] = vy;
        double rep = 0;
        if (py < 0)
            rep = -stiff * py;
        vy += dt * (g + rep);
        py += dt * vy;
    }
*/
    for (int i=0; i<timesteps-1; i++) {
        int offset = i*2;

        problem.AddResidualBlock(ForwardEuler::create(), nullptr, &positions[offset], &positions[offset+2], &velocities[offset+2]);
        problem.AddResidualBlock(SimpleGravity::create(), nullptr, &positions[offset], &velocities[offset], &velocities[offset+2]);
        //problem.AddResidualBlock(SimpleGravityFunctor::create(), nullptr, &velocities[offset], &velocities[offset+2]);
        //problem.AddResidualBlock(WallReflect::create(), nullptr, &positions[offset], &velocities[offset], &velocities[offset+2]);
    }

    positions[1000*2+1] = 15;
    problem.SetParameterBlockConstant(&positions[1000*2]);
    positions[2100*2+1] = 0;
    problem.SetParameterBlockConstant(&positions[2100*2]);

    Solver::Options options;
    options.minimizer_type = LINE_SEARCH;
    options.max_num_iterations = 10000;
    options.line_search_direction_type = LBFGS;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    {
        ofstream ofs("out/x.txt");
        for (int i=0; i<timesteps; i++)
            ofs << i*0.1 << " " << positions[i*2] << " " << positions[i*2+1] << " " << velocities[i*2] << " " << velocities[i*2+1] << endl;

        ofs.close();
    }
    vector<double> residuals;
    problem.Evaluate(Problem::EvaluateOptions(), nullptr, &residuals, nullptr, nullptr);
    {
        ofstream ofs("out/res.txt");
        for (int i=0; i<timesteps-1; i++)
            ofs << i*0.1 << " " << residuals[(i+1)*4] << " " << residuals[(i+1)*4+1] << " " << residuals[(i+1)*4+2] << " " << residuals[(i+1)*4+3] << endl;

        ofs.close();
    }
    exit(0);
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