# Current Work — Primitive Creation Slice (feat/primitive-creation)

- File in flight: none — slice complete; ready for Estimator.
- How far: all 8 PLAN.md todos done. CPU-pure geometry library at `include/primitive_geometry.hpp` (sphere + cube generators with closed-form vertex/facet/edge counts). Runtime `MeshSphereInitializer` and `MeshCubeInitializer` in `src/main.cpp` wrap the generators. `Simulator::addSphere`/`addCube` helpers; `toSnapshot`/`loadScene` round-trip the new shape strings. ImGui `Create > Sphere…` and `Cube…` modals are wired. `scene_format::isKnownShape` accepts sphere/cube; `isReservedShape` is now empty for v1.
- What's tested: 18 test cases / 1262 assertions across two binaries — `ysim_tests` (9/142) and `ysim_primitive_tests` (9/1120). Both `scripts/verify.sh` and `scripts/verify-light.sh` updated to run both binaries. `ysim` GUI binary builds clean.
- What's next: Estimator review. The persistence slice's parked WARNING (no app-level Simulator round-trip) is **still parked** — addressing it requires Q-D and is out of scope for this slice (PLAN.md Non-goals).
