#pragma once
#include <cstdint>

// Backend dispatch tags + global scalar aliases. CUDA is declared only;
// no specialization exists (see DECISIONS A6).
using Index = uint32_t;
using Precision = float;

struct Backend {};
struct CPU : Backend {};
struct CUDA : Backend {};
struct METAL : Backend {};
