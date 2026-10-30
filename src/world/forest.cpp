#include "world/forest.h"

#include <algorithm>

#include "gen/noise.h"

namespace world {

TreeCandidate forestCandidate(const TerrainParams& p, int32_t ci, int32_t cj) {
    uint32_t h = uint32_t(ci) * 0x8DA6B343u ^ uint32_t(cj) * 0xD8163841u ^ p.seed * 0x9E3779B1u ^ 0x7F4A7C15u;
    const auto next = [&h] { h ^= h >> 13; h *= 0x5BD1E995u; h ^= h >> 15; return h; };
    TreeCandidate c;
    c.x = ci * FOREST_CELL + int32_t(next() % FOREST_CELL);
    c.z = cj * FOREST_CELL + int32_t(next() % FOREST_CELL);
    c.roll = float(next() & 0xFFFF) / 65535.0f;
    c.fx = int(next() % 16);
    c.fz = int(next() % 16);
    c.seed = next();
    return c;
}

float forestDensity(const TerrainParams& p, int32_t x, int32_t z, int32_t h) { return forestDensity(p, x, z, h, climateAt(p, x, z)); }

float forestDensity(const TerrainParams& p, int32_t x, int32_t z, int32_t h, const Climate& cl) {
    const float temp = float(temperatureAt(p, cl, h)) / 65536.0f;
    if (temp < float(p.snowBelow) / 65536.0f + 0.05f) return 0.0f;   // снег
    const float rain = float(cl.rain) / float(2 * p.rainMid);
    const auto sst = [](float a, float b, float v) { const float t = std::clamp((v - a) / (b - a), 0.0f, 1.0f); return t * t * (3 - 2 * t); };
    const float grove = float(gen::value2(x, z, 9, p.seed ^ 0xF0E57u)) / 65536.0f;
    return sst(0.05f, 0.5f, rain) * sst(-0.5f, 0.2f, grove) * 0.85f;   // степь - редкие дубы, влажно - рощи
}

}  // namespace world
