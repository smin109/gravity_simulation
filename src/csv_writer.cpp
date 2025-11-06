#include "nbody/csv_writer.hpp"
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>

struct CSVWriter::Impl {
    std::ofstream ofs;
    explicit Impl(const std::string& path) : ofs(path) {}
};

CSVWriter::CSVWriter(const std::string& path) : p(std::make_unique<Impl>(path)) {
    if (!p->ofs.is_open()) {
        throw std::runtime_error("Failed to open CSV file for writing: " + path);
    }
}

CSVWriter::~CSVWriter() = default;

void CSVWriter::header() {
    p->ofs << "t, id, x, y\n";
}

void CSVWriter::dump(double t, const std::vector<Body>& B) {
    for (size_t i{0}; i < B.size(); i++) {
        p->ofs << std::fixed << std::setprecision(6)
               << t << "," << i << "," << B[i].x.x << "," << B[i].x.y << '\n';
    }
}