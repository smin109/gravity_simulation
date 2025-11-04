#include "nbody/csv_writer.hpp"
#include <fstream>
#include <iomanip>

struct CSVWriter::Impl {
    std::ofstream ofs;
    explicit Impl(const std::string& path) : ofs(path) {}
};

CSVWriter::CSVWriter(const std::string& path) : p(new Impl(path)) {}
CSVWriter::~CSVWriter(){ delete p; }

void CSVWriter::header() {
    p->ofs << "t, id, x, y\n";
}

void CSVWriter::dump(double t, const std::vector<Body>& B) {
    for (size_t i{0}; i < B.size(); i++) {
        p -> ofs << std::fixed << std::setprecision(6)
                 << t << "," << i << "," << B[i].x.x << "," << B[i].x.y << '\n';
    }
}