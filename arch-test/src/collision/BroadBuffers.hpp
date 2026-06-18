#pragma once
#include "backend/Backend.hpp"
#include "backend/VectorBase.hpp"
#include "core/Types.hpp"   // BroadCollision (32B, byte-exact mirror)

#include <algorithm>
#include <cstdint>

// Host mirror of bvh.metal QueryFlag (buffer(6) of queryPoints). Two uints:
// stackOverflow (corrupt/too-deep traversal) + collisionOverflow (broad buffer
// full). Each Karras12BVH owns one QueryFlag[1]; SceneBVH reads them after the
// per-frame commit to report under-sizing.
struct QueryFlag {
    uint32_t stackOverflow;
    uint32_t collisionOverflow;
};
static_assert(sizeof(QueryFlag) == 8);

// Broad-phase output buffers, owned by the broad concrete (SceneBVH). No global
// PackedCollisionData in arch-test yet; when the narrow port lands these migrate
// to a shared struct both broad (writes) and narrow (reads) reference. Sized
// clothVerts * approxPerVertex (the original engine's per-vertex contact budget).
// Overflow is reported via QueryFlag.collisionOverflow — the kernel guards
// idx >= maxNumCollisions and never truncates response silently.
template <typename BE, typename PR>
struct BroadBuffers {
    VectorBase<BE, BroadCollision> broadCollisions;   // maxNumCollisions
    VectorBase<BE, uint32_t>       numBroadCollisions; // [1], atomic on GPU
    Index maxNumCollisions = 0;

    void ensure(Index clothVerts, Index approxPerVertex) {
        Index want = std::max<Index>(1, clothVerts * approxPerVertex);
        if (maxNumCollisions != want) {
            maxNumCollisions   = want;
            broadCollisions    = VectorBase<BE, BroadCollision>(want);
            numBroadCollisions = VectorBase<BE, uint32_t>(1, 0u);
        }
    }
};
