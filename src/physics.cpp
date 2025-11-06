#include "nbody/physics.hpp"
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

void DirectAccelerator::compute_acc(std::vector<Body>& B, double G, double soft2) {
    const int N = (int)B.size();
    for (auto& b: B) b.a = {0, 0};

    for (int i{0}; i < N; i++) {
        for (int j{i + 1}; j < N; j++) {
            const double dx = B[j].x.x - B[i].x.x;
            const double dy = B[j].x.y - B[i].x.y;
            const double r2 = dx * dx + dy * dy + soft2;
            const double invr = 1.0 / std::sqrt(r2);
            const double invr3 = invr * invr * invr;
            const double f = G * B[i].m * B[j].m * invr3;
            const double ax = f * dx;
            const double ay = f * dy;
            B[i].a.x += ax / B[i].m;
            B[i].a.y += ay / B[i].m;
            B[j].a.x += -ax / B[j].m;
            B[j].a.y += -ay / B[j].m;
        }
    }
}

void VelocityVerlet::step(SimState& S, IAccelerator& acc, const SimParams& P) {
    auto& B = S.bodies;
    const double dt = P.dt;
    std::vector<Vec2> prevAcc;
    prevAcc.reserve(B.size());

    for (auto& b : B) {
        prevAcc.push_back(b.a);
        b.x.x += b.v.x * dt + 0.5 * b.a.x * dt * dt;
        b.x.y += b.v.y * dt + 0.5 * b.a.y * dt * dt;
    }

    acc.compute_acc(B, P.G, P.soft2);

    for (std::size_t i = 0; i < B.size(); ++i) {
        const Vec2 dv = (prevAcc[i] + B[i].a) * (0.5 * dt);
        B[i].v += dv;
    }
    S.t += dt;
}

void init_disc(SimState& S, const SimParams& P, double centralMass, double vScale) {
    if (P.N <= 0) {
        throw std::invalid_argument("SimParams::N must be positive to initialize the disc");
    }

    S.bodies.assign(P.N, Body{});
    std::mt19937_64 rng(P.seed);
    const double twoPi = 2.0 * std::acos(-1.0);
    std::uniform_real_distribution<double> angleDist(0.0, twoPi);
    std::uniform_real_distribution<double> radiusDist(0.0, 1.0);

    S.bodies[0].m = centralMass;
    S.bodies[0].x = {0, 0};
    S.bodies[0].v = {0, 0};
    S.bodies[0].a = {0, 0};

    for (int i{1}; i < P.N; i++) {
        const double theta = angleDist(rng);
        const double r = P.boxR * std::sqrt(radiusDist(rng));

        const double rx = std::cos(theta);
        const double ry = std::sin(theta);
        const double tx = -ry;
        const double ty = rx;
        const double xPos = rx * r;
        const double yPos = ry * r;

        S.bodies[i].m = 1.0;
        S.bodies[i].x = {xPos, yPos};

        double vmag = 0.0;
        if (r > 0.0) {
            vmag = std::sqrt(P.G * centralMass / r);
        }
        const double speed = vScale * vmag;
        S.bodies[i].v = {tx * speed, ty * speed};
    }

    DirectAccelerator acc;
    acc.compute_acc(S.bodies, P.G, P.soft2);
}