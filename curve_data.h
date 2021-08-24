#pragma once
#include <cstdint>

constexpr int num_vertices = 2000;
constexpr int num_indices = 1000;

extern const float vertex_buffer[2000 * 4];
extern const uint32_t index_buffer[num_indices];