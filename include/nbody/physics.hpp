#pragma once
#include "sim.hpp"

struct IAccelerator {
    virtual ~IAccelerator() = default;
    virtual void compute_acc(std::vector<Body>& B, double G, double soft2) = 0;
};

struct DirectAccelerator : IAccelerator {
    void compute_acc(std::vector<Body>& B, double G, double soft2) override;
};

struct VelocityVerlet {
    void step(SimState& S, IAccelerator& acc, const SimParams& P);
};

void init_disc(SimState& S, const SimParams& P, double centralMass = 500.0, double vScale = 0.6);