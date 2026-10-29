#pragma once
// Кусок мира (Р14, Р17): 32×32×32 вокселя, плотный массив номеров материала по 2 байта (64 КБ).
// Воксель - 1/16 блока (Р13), кусок - 2×2×2 блока. Координаты куска - в кусках: (cx, cy, cz).
#include <array>
#include <cstdint>

namespace world {

constexpr int CHUNK = 32;                 // вокселей в куске по стороне
constexpr int CHUNK_LOG2 = 5;
constexpr int VOXELS_PER_BLOCK = 16;      // Р13

using Material = uint16_t;
enum : Material { AIR = 0, STONE = 1, DIRT = 2, GRASS = 3, SAND = 4, MATERIAL_COUNT };

struct ChunkPos { int32_t x, y, z; };

struct Chunk {
    ChunkPos pos{};
    // порядок: x быстрее всех, потом z, потом y - срез по высоте (слой y) лежит в памяти подряд
    std::array<Material, CHUNK * CHUNK * CHUNK> voxels{};
    Material& at(int x, int y, int z) { return voxels[(y * CHUNK + z) * CHUNK + x]; }
    Material at(int x, int y, int z) const { return voxels[(y * CHUNK + z) * CHUNK + x]; }
};

}  // namespace world
