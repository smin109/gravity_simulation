#include "nbody/physics.hpp"
#include <cmath>
#include <random>

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

void VelocityVerlet::step(simState& S, IAccelerator & acc, const SimParams& P) {
    auto& B = S.boides;
    const double dt = P.dt;
    for (auto& b : B) {
        b.x.x += b.v.x * dt + 0.5 * b.a.x * dt * dt;
        b.x.y += b.v.y * dt + 0.5 * b.a.y * dt * dt;
    }

    acc.compute_acc(B, P.G, P.soft2);

    for (auto& b : B) {
        b.v.x += 0.5 * b.a.x * dt;
        b.v.y += 0.5 * b.a.y * dt;
    }
    S.t += dt;
}

void init_disc(simState& S, const SimParams& P, double centralMass, double vScale) {
    S.boides.assign(P.N, Body{});
    std::mt19937_64 rng(P.seed);
    std::uniform_real_distribution < double > U(-1.0, 1.0);

    S.boides[0].m = centralMass;
    S.boides[0].x = {0, 0};
    S.boides[0].v = {0, 0};
    S.boides[0].a = {0, 0};

    for (int i{1}; i < P.N; i++) {
        double rx = U(rng), ry = U(rng);
        double rnorm = std::sqrt(rx * rx + ry * ry) + 1e-12;
        rx /= rnorm; ry /= rnorm;
        double r = P.boxR * std::pow(std::abs(U(rng)), 0.5);
        S.boides[i].m = 1.0;
        S.boides[i].x = {rx * r, ry * r};

        double tx = -ry, ty = rx;
        double vmag = std::sqrt(P.G * centralMass / (r + 1e-9));
        S.boides[i].v = {rx * (vScale * vmag), ty * (vScale * vmag)};
    }

    DirectAccelerator acc;
    acc.compute_acc(S.boides, P.G, P.soft2);
}