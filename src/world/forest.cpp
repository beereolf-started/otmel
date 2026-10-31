#include "world/forest.h"

#include <algorithm>
#include <unordered_map>
#include <utility>

#include "world/rivers.h"

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

std::vector<float> buildSpriteRegion(const TerrainParams& tp, int rx, int rz, int mode) {
    const bool even = (mode >> 8) & 1, exact = (mode & 0xFF) == 0xFF;
    const int L = exact ? 0 : (mode & 0xFF), step = VOXELS_PER_BLOCK << L, cut = L + 5;
    const int x0 = rx * SPRITE_REGION, z0 = rz * SPRITE_REGION;
    const int pad = exact ? 4 : step + 2;
    RiverSet rivers;
    rivers.build(tp, x0 - pad, z0 - pad, x0 + SPRITE_REGION + pad, z0 + SPRITE_REGION + pad);
    const auto at = [&](int x, int z) { return reliefAt(tp, x, z, slowAt(tp, x, z, 0), 0, &rivers); };
    std::unordered_map<uint64_t, std::pair<int32_t, bool>> nodes;   // узлы плитки: высота дна, мокрый
    const auto node = [&](int64_t i, int64_t j) {
        const uint64_t key = uint64_t(uint32_t(int32_t(i))) << 32 | uint32_t(int32_t(j));
        if (auto it = nodes.find(key); it != nodes.end()) return it->second;
        const int32_t x = int32_t(i * step), z = int32_t(j * step);
        const Relief r = reliefAt(tp, x, z, slowAt(tp, x, z, cut), cut, &rivers);
        return nodes[key] = {r.height, waterLevelAt(tp, r) > r.height};
    };
    const auto fdiv = [](int64_t v, int64_t d) { return v >= 0 ? v / d : -((-v + d - 1) / d); };
    constexpr int CELL = FOREST_CELL;
    std::vector<float> inst;
    for (int64_t cj = fdiv(z0, CELL); cj <= fdiv(z0 + SPRITE_REGION - 1, CELL); ++cj)
        for (int64_t ci = fdiv(x0, CELL); ci <= fdiv(x0 + SPRITE_REGION - 1, CELL); ++ci) {
            if (even && ((ci & 1) || (cj & 1))) continue;
            const TreeCandidate tc = forestCandidate(tp, int32_t(ci), int32_t(cj));
            if (tc.x < x0 || tc.z < z0 || tc.x >= x0 + SPRITE_REGION || tc.z >= z0 + SPRITE_REGION) continue;
            float base;   // низ дерева, блоки
            if (exact) {
                const Relief r = at(tc.x, tc.z);
                if (r.water > r.height) continue;
                if (tc.roll >= forestDensity(tp, tc.x, tc.z, r.height)) continue;
                bool bad = false;
                int32_t lo = r.height, hi = r.height;
                for (int dz = -3; dz <= 3 && !bad; dz += 3)
                    for (int dx = -3; dx <= 3; dx += 3) {
                        if (dx == 0 && dz == 0) continue;
                        const Relief q = at(tc.x + dx, tc.z + dz);
                        if (q.water > q.height) { bad = true; break; }
                        lo = std::min(lo, q.height);
                        hi = std::max(hi, q.height);
                    }
                if (bad || hi - lo > 4) continue;   // у воды или круто (вблизи - перепад больше 3 на ±2)
                int32_t g0 = r.height;
                for (const auto [dx, dz] : {std::pair{1, 0}, {0, 1}, {1, 1}}) g0 = std::min(g0, at(tc.x + dx, tc.z + dz).height);
                base = float(g0 + 1) - 0.2f;
            } else {
                const float fx = float(tc.x) + (tc.fx + 0.5f) / 16.0f, fz = float(tc.z) + (tc.fz + 0.5f) / 16.0f;
                const int64_t i = fdiv(tc.x, step), j = fdiv(tc.z, step);
                const float tx = (fx - float(i * step)) / float(step), tz = (fz - float(j * step)) / float(step);
                const auto a = node(i, j), b = node(i, j + 1), c = node(i + 1, j + 1), d = node(i + 1, j);
                float h;
                int32_t lo, hi;
                if (tz >= tx) {   // треугольник (i, j) - (i, j+1) - (i+1, j+1)
                    if (a.second || b.second || c.second) continue;
                    h = float(a.first) + tz * float(b.first - a.first) + tx * float(c.first - b.first);
                    lo = std::min({a.first, b.first, c.first}); hi = std::max({a.first, b.first, c.first});
                } else {          // треугольник (i, j) - (i+1, j+1) - (i+1, j)
                    if (a.second || c.second || d.second) continue;
                    h = float(a.first) + tx * float(d.first - a.first) + tz * float(c.first - d.first);
                    lo = std::min({a.first, c.first, d.first}); hi = std::max({a.first, c.first, d.first});
                }
                if (hi - lo > step) continue;   // круче 45°
                if (tc.roll >= forestDensity(tp, tc.x, tc.z, int32_t(h))) continue;
                base = h + 1.0f - 0.5f;   // верх верхнего вокселя; полблока в землю - на плитке без шва
            }
            const float fr = float(tc.seed & 0xFFFF) / 65535.0f;
            inst.insert(inst.end(), {(float(tc.x) + (tc.fx + 0.5f) / 16.0f) / 16.0f, base / 16.0f, (float(tc.z) + (tc.fz + 0.5f) / 16.0f) / 16.0f,
                                     (10.0f + 3.0f * fr) / 16.0f, (8.0f + 3.0f * fr) * (even ? 1.6f : 1.0f) / 16.0f,
                                     float(tc.seed & 0x7FFF), 0.0f, 0.0f});
        }
    return inst;
}

}  // namespace world
