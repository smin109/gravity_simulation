#pragma once
#include <vector>
#include <random>
#include "body.hpp"

struct SimParams {
    int N{200};
    double G{1.0};
    double soft2{1e-2};
    double dt{1e-2};
    int steps{20000};
    int saveEvery{10};
    double boxR{50.0};
    uint64_t seed{42};
};

struct SimState {
    std::vector<Body> bodies;
    double t{0.0};
};