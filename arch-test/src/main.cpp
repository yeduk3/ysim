// arch_runner — headless entry point. Builds the basic scene via the
// setupBasicScene free function and steps it with the Runner (blueprint
// minimum goal). No GL / ImGui.
#include "backend/MemoryPool.hpp"
#include "runner/Runner.hpp"

#include <cstdlib>

int main(int argc, char** argv) {
    int frames = (argc > 1) ? std::atoi(argv[1]) : 300;

    GlobalAutoAllocator<METAL>::globalInitialize(size_t(1) << 28);

    Runner<METAL, Precision> runner(frames);
    bool single = runner.run();          // single-sim PASS path (back-compat)
    bool multi  = runner.runMulti();     // N coexisting Simulators (O2)
    return (single && multi) ? 0 : 1;
}
