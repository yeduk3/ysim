#pragma once
#include "sim/Simulator.hpp"
#include "core/Scene.hpp"
#include "ysim_paths.hpp"

// The basic/default scene as a FREE FUNCTION (blueprint goal 2, the minimum
// goal). Recipe is the exact default-scene from src/main.cpp 18282-18288
// (DECISIONS C16): id 0 cloth, id 1 human (Float), id 2 ground (Float).
// addX only queues (CPU); GPU work happens in Simulator::initialize.
template <typename BE, typename PR>
void setupBasicScene(Simulator<BE, PR>& sim) {
    // id 0 — TriangularCloth, 50x50 XZ grid, dropped above the human.
    ObjectDesc cloth;
    cloth.kind = ObjectDesc::Kind::GridCloth;
    cloth.behavior = BehaviorType::TriangularCloth;
    cloth.shape = ShapeType::Mesh;
    cloth.gridN = 50;
    cloth.sizeWorld = 1.0f;
    cloth.origin = tinym::vec3(0.0f, 1.25f, 0.0f);
    cloth.cloth = ClothBehaviorParams<float>{1e5f, 1e5f, 2e5f, 0.01f}; // stretch,shear,bend,thickness
    sim.add(cloth);

    // id 1 — Human.obj, Float (one-way collider). OBJ load deferred this pass.
    ObjectDesc human;
    human.kind = ObjectDesc::Kind::FileMesh;
    human.behavior = BehaviorType::Float;
    human.shape = ShapeType::Mesh;
    human.filePath = ysim_paths::assetFile("Human.obj");
    human.origin = tinym::vec3(0.0f, 0.35f, 0.0f);
    human.scale = 0.04f;
    sim.add(human);

    // id 2 — XZ ground plane, Float + static, 100m x 100m.
    ObjectDesc ground;
    ground.kind = ObjectDesc::Kind::Ground;
    ground.behavior = BehaviorType::Float;
    ground.shape = ShapeType::Plane;
    ground.isStatic = true;
    ground.sizeWorld = 50.0f;
    ground.origin = tinym::vec3(0.0f, 0.0f, 0.0f);
    sim.add(ground);
}
