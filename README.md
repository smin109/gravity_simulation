# gravity-sim

C++/CMake 기반 2D 중력 N-body 시뮬레이터.  
O(N^2) 직접력 + Velocity-Verlet, CSV 출력.

## Build
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
