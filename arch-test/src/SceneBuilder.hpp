#include "Scene.hpp"
#include "ClothDesc.hpp"
#include <vector>


struct SceneBuilder {
    
    // Add object //

    std::vector<ClothDesc> clothDescs;
    void add(ClothDesc& clothDesc) { return; }


    // Define System

    void setSystem();

    // Define CD Pipeline

    void setCDPipeline();

    // Build //

    Scene build();
};
