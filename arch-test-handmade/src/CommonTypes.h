#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

// NOTE: C++23's <stdfloat> / std::float32_t is NOT shipped by AppleClang's
// libc++ (verified with Apple clang 21.0.0, 2026-07: <stdfloat> file not found).
// Fall back to plain float. Restore std::float32_t once libc++ provides <stdfloat>.
// #include <stdfloat>

#include <cstdint>

#ifndef YSIM_DOUBLE_PRECISION
using Real = float;   // would be std::float32_t if <stdfloat> were available
#else
using Real = double;
#endif

using UInt = std::uint32_t;
using Int  = std::int32_t;

#endif // COMMON_TYPES_HPP
