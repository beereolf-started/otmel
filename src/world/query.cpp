#include "world/query.h"

#include <algorithm>

#include "world/caves.h"

namespace world {
namespace {
int32_t floorDiv(int32_t a, int32_t b) { return a >= 0 ? a / b : -(((-a) + b - 1) / b); }
}  // namespace

VoxelQuery::Slot& VoxelQuery::slotFor(int32_t cx, int32_t cz) {
    for (Slot& s : slots_)
        if (s.cx == cx && s.cz == cz) return s;
    Slot& s = slots_[next_];
    next_ = (next_ + 1) % SLOTS;
    s.cx = cx;
    s.cz = cz;
    s.cy = INT32_MIN;
    columnHeights(p_, cx, cz, s.hm);
    soilCut(p_, cx, cz, s.hm, s.cut);
    return s;
}

Material VoxelQuery::at(int32_t vx, int32_t vy, int32_t vz) {
    if (edits_) {   // правка главнее всего: её видит и сетка, и столкновения
        if (!snap_ || snapVer_ != edits_->version()) snap_ = edits_->snapshot(&snapVer_);
        const auto it = snap_->find(editKey(vx, vy, vz));
        if (it != snap_->end()) return it->second.m;
    }
    Slot& s = slotFor(floorDiv(vx, CHUNK), floorDiv(vz, CHUNK));
    const int32_t ncy = floorDiv(vy, CHUNK);
    if (ncy != s.cy) {
        s.cy = ncy;
        s.caves = caveAir(p_, ChunkPos{s.cx, s.cy, s.cz}, s.hm, s.air, s.cut);
    }
    const int lx = vx - s.cx * CHUNK, lz = vz - s.cz * CHUNK, ly = vy - s.cy * CHUNK;
    const int32_t vcut = s.cut[lz + 1][lx + 1];
    const int32_t top = s.hm.h[lz + 1][lx + 1] - vcut;
    // Р82: снежный слой - воксель над верхом столбца; для столкновений он твёрдый целиком
    if (vy == top + 1 && s.hm.snow[lz + 1][lx + 1]) return SNOW;
    Cover cv = s.hm.cover[lz][lx];
    cv.soil = int16_t(std::max(0, int(cv.soil) - vcut));
    const Material m = columnMaterial(top, cv, vy, s.hm.water[lz + 1][lx + 1], s.hm.rock, vx, vz);
    if (s.caves && ly >= 0 && ly < 64 && ((s.air[lz + 1][lx + 1] >> ly) & 1)) return AIR;
    return m;
}

}  // namespace world
