#pragma once
#include <string>
#include "sim.hpp"

struct CSVWriter {
    explicit CSVWriter(const std::string& path);
    ~CSVWriter();
    void header();
    void dump(double t, const std::vector<Body>& B);
private:
    struct Impl; Impl* p;
};
