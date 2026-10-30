#include "world/flow.h"

#include <algorithm>

#include "world/rivers.h"

namespace world {

void flowField(const TerrainParams& p, int32_t bx0, int32_t bz0, int size, std::vector<float>& out,
               std::vector<float>* ground, std::vector<float>* surface) {
    out.assign(size_t(size) * size * 2, 0.0f);
    if (ground) ground->assign(size_t(size) * size, 0.0f);
    if (surface) surface->assign(size_t(size) * size, -1e9f);
    // набор рек - кусками по 64 блока: на кусок свой (как у плиток), звеньев в нём немного
    constexpr int PIECE = 64;
    for (int pz = 0; pz < size; pz += PIECE)
        for (int px = 0; px < size; px += PIECE) {
            const int32_t x0 = (bx0 + px) * VOXELS_PER_BLOCK, z0 = (bz0 + pz) * VOXELS_PER_BLOCK;
            RiverSet rs;
            rs.build(p, x0, z0, x0 + PIECE * VOXELS_PER_BLOCK, z0 + PIECE * VOXELS_PER_BLOCK);
            for (int j = pz; j < std::min(size, pz + PIECE); ++j)
                for (int i = px; i < std::min(size, px + PIECE); ++i) {
                    const int32_t x = (bx0 + i) * VOXELS_PER_BLOCK + VOXELS_PER_BLOCK / 2;
                    const int32_t z = (bz0 + j) * VOXELS_PER_BLOCK + VOXELS_PER_BLOCK / 2;
                    const size_t k = size_t(j) * size + i;
                    // дно и вода столбца - для автомата потревоженной воды (шаг 3)
                    if (ground || surface) {
                        const Relief r = reliefAt(p, x, z, slowAt(p, x, z), 0, &rs);
                        if (ground) (*ground)[k] = float(r.height + 1) / VOXELS_PER_BLOCK;
                        if (surface && r.water > r.height) (*surface)[k] = float(r.water + 1) / VOXELS_PER_BLOCK;
                    }
                    if (rs.segs.empty()) continue;
                    const RiverSet::Current c = rs.current(p, x, z);
                    if (!c.inRiver) continue;
                    const float sp = float(c.speed) / 65536.0f;
                    out[k * 2] = float(c.dirX) / 65536.0f * sp;
                    out[k * 2 + 1] = float(c.dirZ) / 65536.0f * sp;
                }
        }
}

}  // namespace world
