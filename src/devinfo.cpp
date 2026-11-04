// Подсказки разработчика (devinfo.h): луч из камеры и разбор координат перехода.
#include "devinfo.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>

#include "world/caves.h"
#include "world/chunk.h"
#include "world/rock_table.h"
#include "world/terrain.h"

namespace dev {
namespace {

using world::CHUNK;
using world::VOXELS_PER_BLOCK;

int32_t floorDiv(int32_t a, int32_t b) { return a >= 0 ? a / b : -(((-a) + b - 1) / b); }

/** Имена материалов, кроме пород: по номеру из world::chunk.h. */
const char* plainName(world::Material m) {
    switch (m) {
        case world::AIR: return "воздух";
        case world::STONE: return "камень";
        case world::DIRT: return "земля";
        case world::GRASS: return "дёрн";
        case world::SAND: return "песок";
        case world::SNOW: return "снег";
        case world::WATER: return "вода";
        case world::BARK: case world::BARK2: return "кора";
        case world::WOOD: return "древесина";
        case world::LEAVES: case world::LEAVES2: return "листва";
        default: return "неизвестный материал";
    }
}

std::string nameOf(world::Material m) {
    if (m >= world::ROCK_FIRST && m <= world::ROCK_LAST) return world::rockDef(int(m) - world::ROCK_FIRST).name;
    return plainName(m);
}

/** Главные минералы породы - чтобы состав было видно, а не только имя (Р78). */
std::string mineralsOf(world::Material m) {
    if (m < world::ROCK_FIRST || m > world::ROCK_LAST) return {};
    static const char* MN[world::MINERALS] = {"кварц", "полевой шпат", "слюда", "тёмноцветные",
                                              "кальцит", "доломит", "глина", "органика"};
    const world::Composition& c = world::rockDef(int(m) - world::ROCK_FIRST).comp;
    int order[world::MINERALS];
    for (int i = 0; i < world::MINERALS; ++i) order[i] = i;
    std::sort(order, order + world::MINERALS, [&](int a, int b) { return c.m[a] > c.m[b]; });
    std::ostringstream s;
    for (int k = 0; k < 3 && c.m[order[k]] > 0.02f; ++k) {
        if (k) s << ", ";
        s << MN[order[k]] << ' ' << int(c.m[order[k]] * 100.0f + 0.5f) << '%';
    }
    return s.str();
}

/** Куски генерации по одному: столбец высот и маска пещер считаются раз на кусок. */
struct ChunkCache {
    const world::TerrainParams& p;
    world::ColumnHeights hm{};
    uint64_t air[CHUNK + 2][CHUNK + 2]{};
    uint8_t cut[CHUNK + 2][CHUNK + 2]{};   // Р79: срез земли у выхода шахты
    int32_t cx = INT32_MIN, cz = INT32_MIN, cy = INT32_MIN;
    bool caves = false;
    int built = 0;

    explicit ChunkCache(const world::TerrainParams& par) : p(par) {}

    /** Материал вокселя мира; INT32_MIN в built - исчерпан запас кусков. */
    world::Material at(int32_t vx, int32_t vy, int32_t vz, int maxChunks) {
        const int32_t ncx = floorDiv(vx, CHUNK), ncz = floorDiv(vz, CHUNK), ncy = floorDiv(vy, CHUNK);
        if (ncx != cx || ncz != cz) {
            if (built >= maxChunks) return world::AIR;
            world::columnHeights(p, ncx, ncz, hm);
            world::soilCut(p, ncx, ncz, hm, cut);
            cx = ncx;
            cz = ncz;
            cy = INT32_MIN;
            ++built;
        }
        if (ncy != cy) {
            cy = ncy;
            caves = world::caveAir(p, world::ChunkPos{cx, cy, cz}, hm, air, cut);
        }
        const int lx = vx - cx * CHUNK, lz = vz - cz * CHUNK, ly = vy - cy * CHUNK;
        // Р79: верх с учётом среза, а подошва земли на месте - как и в сетке
        world::Cover cv = hm.cover[lz][lx];
        const int vcut = cut[lz + 1][lx + 1];
        cv.soil = int16_t(std::max(0, int(cv.soil) - vcut));
        const world::Material m = world::columnMaterial(hm.h[lz + 1][lx + 1] - vcut, cv, vy,
                                                        hm.water[lz + 1][lx + 1], hm.rock, vx, vz);
        if (caves && ly >= 0 && ly < 64 && ((air[lz + 1][lx + 1] >> ly) & 1)) return world::AIR;
        return m;
    }
};

}  // namespace

float groundAt(const world::TerrainParams& p, float x, float z) {
    static world::ColumnHeights hm;
    const int32_t vx = int32_t(std::lround(x * VOXELS_PER_BLOCK)), vz = int32_t(std::lround(z * VOXELS_PER_BLOCK));
    world::columnHeights(p, floorDiv(vx, CHUNK), floorDiv(vz, CHUNK), hm);
    const int lx = vx - floorDiv(vx, CHUNK) * CHUNK, lz = vz - floorDiv(vz, CHUNK) * CHUNK;
    // Р79: поверхность с учётом среза земли у выхода шахты
    static uint8_t cut[CHUNK + 2][CHUNK + 2];
    world::soilCut(p, floorDiv(vx, CHUNK), floorDiv(vz, CHUNK), hm, cut);
    const int32_t h = std::max(hm.h[lz + 1][lx + 1] - cut[lz + 1][lx + 1], hm.water[lz + 1][lx + 1] == world::NO_WATER
                                                         ? INT32_MIN
                                                         : hm.water[lz + 1][lx + 1]);
    return float(h) / float(VOXELS_PER_BLOCK);
}

Aim aimedAt(const world::TerrainParams& p, glm::vec3 eye, float yaw, float pitch, float maxBlocks) {
    Aim out;
    const glm::vec3 fwd(std::cos(pitch) * std::cos(yaw), std::sin(pitch), std::cos(pitch) * std::sin(yaw));
    ChunkCache cache(p);
    const int steps = int(maxBlocks * VOXELS_PER_BLOCK);      // шаг - воксель, то есть 1/16 блока
    const glm::vec3 start = eye * float(VOXELS_PER_BLOCK);
    for (int i = 1; i <= steps; ++i) {
        const glm::vec3 v = start + fwd * float(i);
        const int32_t vx = int32_t(std::floor(v.x)), vy = int32_t(std::floor(v.y)), vz = int32_t(std::floor(v.z));
        const world::Material m = cache.at(vx, vy, vz, 64);
        if (m == world::AIR) continue;
        out.hit = true;
        out.name = nameOf(m);
        out.detail = mineralsOf(m);
        out.block = glm::ivec3(floorDiv(vx, VOXELS_PER_BLOCK), floorDiv(vy, VOXELS_PER_BLOCK),
                               floorDiv(vz, VOXELS_PER_BLOCK));
        out.distance = float(i) / float(VOXELS_PER_BLOCK);
        return out;
    }
    return out;
}

bool parseGoto(const std::string& s, glm::vec3& out, bool& hasY) {
    std::istringstream in(s);
    double a = 0, b = 0, c = 0;
    if (!(in >> a >> b)) return false;
    if (in >> c) {
        out = glm::vec3(float(a), float(b), float(c));
        hasY = true;
    } else {
        out = glm::vec3(float(a), 0.0f, float(b));
        hasY = false;
    }
    return true;
}

}  // namespace dev
