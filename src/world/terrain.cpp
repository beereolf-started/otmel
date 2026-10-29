#include "world/terrain.h"

#include <algorithm>

#include "gen/noise.h"

namespace world {

void columnHeights(const TerrainParams& p, int32_t chunkX, int32_t chunkZ, ColumnHeights& out) {
    const int32_t x0 = chunkX * CHUNK - 1, z0 = chunkZ * CHUNK - 1;
    out.minH = INT32_MAX;
    out.maxH = INT32_MIN;
    for (int z = 0; z < CHUNK + 2; ++z) {
        for (int x = 0; x < CHUNK + 2; ++x) {
            const fx::q16 n = gen::fbm2(x0 + x, z0 + z, p.firstCellLog2, p.octaves, p.seed);
            // высота = база + шум·размах; шум в Q16, произведение сдвигом возвращаем в целые воксели
            const int32_t h = p.baseHeight + static_cast<int32_t>((static_cast<int64_t>(n) * p.amplitude) >> fx::SHIFT);
            out.h[z][x] = h;
            out.minH = std::min(out.minH, h);
            out.maxH = std::max(out.maxH, h);
        }
    }
}

void fillChunk(const ColumnHeights& heights, Chunk& c) {
    const int32_t y0 = c.pos.y * CHUNK;
    constexpr int32_t DIRT_DEPTH = 3 * VOXELS_PER_BLOCK;
    for (int z = 0; z < CHUNK; ++z) {
        for (int x = 0; x < CHUNK; ++x) {
            const int32_t top = heights.h[z + 1][x + 1];
            for (int y = 0; y < CHUNK; ++y) {
                const int32_t wy = y0 + y;
                Material m = AIR;
                if (wy == top) m = GRASS;
                else if (wy < top && wy >= top - DIRT_DEPTH) m = DIRT;
                else if (wy < top) m = STONE;
                c.at(x, y, z) = m;
            }
        }
    }
}

uint64_t checksum(const Chunk& c) {
    // FNV-1a по всем вокселям: любое расхождение хоть в одном вокселе меняет сумму
    uint64_t h = 1469598103934665603ull;
    for (Material m : c.voxels) { h ^= m; h *= 1099511628211ull; }
    return h;
}

}  // namespace world
