#include "CollisionDetectionPipeline.hpp"

// Simulator struct handles a loop in the simulation.
template <typename CollisionDetectionPipeline,
         typename System>
struct Simulator {
    CollisionDetectionPipeline cdPipeline;
    System system;

    
    void initialize() {
        SceneBuilder

        BVHBuilder
        CollisionDetectionPipeline(Scene, BVHBuilder)

        SystemBuilder


        Simulator
    }

    // The inheritance should not included in this level.
    void step() {
        for(int substep = 0; substep < substeps; substep++) {
            // discrete collision detection
            cdPipeline.dcd();

            // force accumulation
            system.accumulate();

            // integration
            system.integration();

            // continuous collision detection
            cdPipeline.ccd();

            // recovery
            system.recoveryPenetration();
        }
    }

    void loop() {
        for(int frame = 0; frame < maxFrame; frame++) {

            step();
        };
    }
};
