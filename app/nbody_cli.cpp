#include <iostream>
#include <string>
#include <cstring>
#include "nbody/physics.hpp"
#include "nbody/csv_writer.hpp"

int main(int argc, char** argv) {
    SimParams P;
    std::string outPath = "out.csv";

    for (int i{1};i < argc; i++) {
        if (!std::strcmp(argv[i], "--n") && i + 1 < argc) P.N = std::stoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--steps") && i + 1 < argc) P.steps = std::stoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--dt") && i + 1 < argc) P.dt = std::stod(argv[++i]);
        else if (!std::strcmp(argv[i], "--G") && i + 1 < argc) P.G = std::stod(argv[++i]);
        else if (!std::strcmp(argv[i], "--soft2") && i + 1 < argc) P.soft2 = std::stod(argv[++i]);
        else if (!std::strcmp(argv[i], "--save") && i + 1 < argc) P.saveEvery = std::stoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--seed") && i + 1 < argc) P.seed = std::stoull(argv[++i]);
        else if (!std::strcmp(argv[i], "--out") && i + 1 < argc) outPath = argv[++i];
        else if (!std::strcmp(argv[i], "--boxR") && i + 1 < argc) P.boxR = std::stod(argv[++i]);
        else if (!std::strcmp(argv[i], "--help")) {
            std::cout <<
            "Usage: nbody_cli [options]\n"
            "  --n INT          (default 200)\n"
            "  --steps INT      (default 20000)\n"
            "  --dt FLOAT       (default 0.01)\n"
            "  --G FLOAT        (default 1.0)\n"
            "  --soft2 FLOAT    (default 0.01)\n"
            "  --save INT       (save every k steps, default 10)\n"
            "  --seed UINT64    (default 42)\n"
            "  --boxR FLOAT     (default 50.0)\n"
            "  --out PATH       (default out.csv)\n";
            return 0;
        }
    }

    SimState S;
    init_disc(S, P, 500.0, 0.6);

    DirectAccelerator acc;
    VelocityVerlet integrator;

    CSVWriter writer(outPath);
    writer.header();
    writer.dump(S.t, S.bodies);

    for (int s{1}; s <= P.steps; s++) {
        integrator.step(S, acc, P);
        if (s % P.saveEvery == 0) writer.dump(S.t, S.bodies);
    }

    std::cerr << "완료 : " << outPath << " 생성\n";
    return 0;
}