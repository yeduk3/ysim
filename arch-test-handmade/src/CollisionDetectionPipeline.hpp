#ifndef COLLISION_DETECTION_PIPELINE_HPP
#define COLLISION_DETECTION_PIPELINE_HPP

#include <concepts>

template <class BP, class NP, class SCENE>
concept BroadNarrowCD = requires(BP& bp, NP& np, SCENE& scene) {
    { bp.build(scene) } -> std::same_as<void>;
    { bp.refit(scene) } -> std::same_as<void>;
    { bp.enlargeTrajectory(scene) } -> std::same_as<void>;
    { bp.detect(scene) } -> std::same_as<void>;
    { np.detect(scene) } -> std::same_as<void>;
};

template <class CDP, class SCENE>
concept CombinedCD = requires(CDP& cdp, SCENE& scene) {
    { cdp.build(scene) } -> std::same_as<void>;
    { cdp.detect(scene) } -> std::same_as<void>;
};

template <class BP, class NP, class SCENE, class CDP>
concept CollisionDetectionPipeline = 
    BroadNarrowCD<BP, NP, SCENE> || 
    CombinedCD<CDP, SCENE>;




#endif // !COLLISION_DETECTION_PIPELINE_HPP
