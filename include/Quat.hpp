#ifndef YSIM_QUAT_HPP
#define YSIM_QUAT_HPP

// Bare quaternion aggregate. {w, x, y, z} layout (scalar-first). Order
// matches the on-disk schema. Identity is the default.
//
// Helper functions (quatNormalize, operator*, quatFromAxisAngle,
// quatToAxisAngle, quatFromEulerXYZ, quatToEulerXYZ, quatConjugate,
// rotateVector, etc.) live in src/main.cpp around line 1556+. They
// stay there for now; migration to a math header is deferred to the
// source-file split slice.
struct Quat {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

#endif
