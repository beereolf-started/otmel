#include "world/plants.h"

#include <algorithm>
#include <cmath>

#include "gen/noise.h"

namespace world {
namespace {

uint32_t mix(uint32_t h) { h ^= h >> 15; h *= 0x2C1B3C6Du; h ^= h >> 12; h *= 0x297A2D39u; h ^= h >> 15; return h; }
struct R {
    uint32_t h;
    uint32_t next() { h = mix(h + 0x9E3779B9u); return h; }
    float f() { return float(next() & 0xFFFFFF) / 16777216.0f; }
    int i(int a, int b) { return a + int(next() % uint32_t(b - a + 1)); }
};
void put(std::vector<PlantVoxel>& o, float x, float y, float z, Material m) {
    const int ix = int(std::floor(x)), iy = int(std::floor(y)), iz = int(std::floor(z));
    if (ix < 0 || ix > 31 || iz < 0 || iz > 31 || iy < 0 || iy > 31) return;
    o.push_back({int8_t(ix), int8_t(iy), int8_t(iz), m});
}
// стебель: от (x, z) вверх на h, отклоняется наружу (dx, dz) и под конец клонится (изгиб под тяжестью)
void stalk(std::vector<PlantVoxel>& o, float x, float z, int h, float dx, float dz, float bend, Material m, Material top = AIR) {
    for (int y = 0; y < h; ++y) {
        const float t = float(y) / float(std::max(1, h));
        const float off = t + bend * t * t * t * 2.0f;
        put(o, x + dx * off * float(h) * 0.5f, float(y) - bend * t * t * float(h) * 0.25f, z + dz * off * float(h) * 0.5f, m);
    }
    if (top != AIR) put(o, x + dx * (1 + bend * 2) * float(h) * 0.5f, float(h) - bend * float(h) * 0.25f, z + dz * (1 + bend * 2) * float(h) * 0.5f, top);
}

}  // namespace

const char* plantName(PlantKind k) {
    static const char* N[] = {"", "трава", "высокая трава", "клевер", "одуванчик", "колокольчик", "смолёвка", "ромашка",
                              "крапива", "папоротник", "мох", "камыш", "низкая трава"};
    return N[int(k)];
}

PlantKind plantAt(const TerrainParams& p, int32_t x, int32_t z, Material top, float forest, bool wetNear, uint32_t salt) {
    uint32_t h = mix(uint32_t(x) * 0x8DA6B343u ^ uint32_t(z) * 0xD8163841u ^ p.seed * 0x9E3779B1u ^ 0x91A47u ^ salt * 0x27D4EB2Du);
    const float u = float(h & 0xFFFF) / 65536.0f;
    if (wetNear) return u < 0.5f ? PlantKind::REED : (isGrass(top) ? PlantKind::TALL_GRASS : PlantKind::NONE);
    if (!isGrass(top)) return PlantKind::NONE;
    const float patch = float(gen::value2(x, z, 4, p.seed ^ 0x7A7C4u)) / 65536.0f;   // пятна по 16 блоков
    const float patch2 = float(gen::value2(x, z, 3, p.seed ^ 0x1CE5u)) / 65536.0f;
    if (forest > 0.25f) {   // подлесок
        if (u < 0.22f) return PlantKind::FERN;
        if (u < 0.30f && salt == 0) return PlantKind::MOSS;
        if (u < 0.37f && patch > 0.2f) return PlantKind::NETTLE;
        if (u < 0.70f) return PlantKind::SHORT_GRASS;
        return PlantKind::NONE;
    }
    // луг: по пояс
    if (u < 0.46f) return PlantKind::GRASS;
    if (u < 0.66f) return patch > -0.1f ? PlantKind::TALL_GRASS : PlantKind::GRASS;
    if (u < 0.74f) return patch < -0.15f ? PlantKind::CLOVER : PlantKind::GRASS;
    if (u < 0.82f) {
        if (patch2 > 0.3f) return PlantKind::DANDELION;
        if (patch2 > 0.0f) return PlantKind::CHAMOMILE;
        if (patch2 > -0.3f) return PlantKind::CAMPION;
        return PlantKind::BLUEBELL;
    }
    if (u < 0.85f && patch > 0.4f) return PlantKind::NETTLE;
    return PlantKind::NONE;
}

void stampPlant(PlantKind k, int v, std::vector<PlantVoxel>& o) {
    o.clear();
    R r{mix(uint32_t(k) * 7919u + uint32_t(v) * 104729u + 17u)};
    const float cx = 16, cz = 16;
    const auto bladeSet = [&](int n, int hmin, int hmax, float spread, float lean, float seeds) {
        for (int i = 0; i < n; ++i) {
            const float a = r.f() * 6.283f, d = r.f() * spread;
            const float x = cx + std::cos(a) * d, z = cz + std::sin(a) * d;
            const float dx = std::cos(a) * lean * (0.4f + r.f()), dz = std::sin(a) * lean * (0.4f + r.f());
            const int h = r.i(hmin, hmax);
            const Material m = r.f() < 0.08f ? HAY : (r.f() < 0.5f ? GRASS_BLADE : GRASS_BLADE2);
            const bool head = r.f() < seeds;
            stalk(o, x, z, h, dx, dz, r.f() * 0.6f, m, head ? REED : AIR);
            if (head) stalk(o, x + dx * h * 0.5f, z + dz * h * 0.5f, 3, 0, 0, 0, REED, AIR);
        }
    };
    const auto flower = [&](Material head, bool ring) {
        const float a = r.f() * 6.283f, d = r.f() * 5.0f;
        const float x = cx + std::cos(a) * d, z = cz + std::sin(a) * d;
        const int h = r.i(8, 14);
        stalk(o, x, z, h, 0.03f, 0.02f, 0.1f, GRASS_BLADE);
        const float hx = x + 0.03f * h * 0.5f, hz = z + 0.02f * h * 0.5f;
        if (ring) {   // ромашка: белые лепестки кругом, жёлтая середина
            for (int a2 = -1; a2 <= 1; ++a2) for (int b = -1; b <= 1; ++b) put(o, hx + a2, float(h), hz + b, (a2 || b) ? FLOWER_W : FLOWER_Y);
        } else {
            put(o, hx, float(h), hz, head); put(o, hx + 1, float(h), hz, head); put(o, hx, float(h), hz + 1, head); put(o, hx, float(h + 1), hz, head);
            put(o, hx + 1, float(h + 1), hz + 1, head);
        }
        for (int l = 0; l < 4; ++l) {   // прикорневые листья
            const float la = r.f() * 6.283f;
            for (int s = 1; s < 4; ++s) put(o, x + std::cos(la) * s, s > 2 ? 1.0f : 0.0f, z + std::sin(la) * s, GRASS_BLADE2);
        }
    };
    switch (k) {
        case PlantKind::GRASS: bladeSet(r.i(12, 18), 9, 16, 3.5f, 0.10f, 0.15f); break;            // по пояс
        case PlantKind::TALL_GRASS: bladeSet(r.i(14, 22), 12, 19, 4.0f, 0.12f, 0.35f); break;
        case PlantKind::SHORT_GRASS: bladeSet(r.i(6, 11), 4, 8, 3.0f, 0.10f, 0.0f); break;
        case PlantKind::CLOVER: {
            for (int i = r.i(10, 16); i > 0; --i) {
                const float x = cx + (r.f() - 0.5f) * 14, z = cz + (r.f() - 0.5f) * 14, y = float(r.i(0, 2));
                put(o, x, y, z, CLOVER); put(o, x + 1, y, z, CLOVER); put(o, x, y, z + 1, CLOVER); put(o, x - 1, y, z, CLOVER);
            }
            for (int i = r.i(2, 4); i > 0; --i) { const float x = cx + (r.f() - 0.5f) * 10, z = cz + (r.f() - 0.5f) * 10; stalk(o, x, z, 4, 0, 0, 0, CLOVER, r.f() < 0.5f ? FLOWER_W : FLOWER_R); }
            bladeSet(4, 4, 8, 5.0f, 0.1f, 0.0f);
            break;
        }
        case PlantKind::DANDELION: for (int i = r.i(2, 4); i > 0; --i) flower(FLOWER_Y, false); bladeSet(5, 5, 10, 4.0f, 0.1f, 0.0f); break;
        case PlantKind::BLUEBELL: for (int i = r.i(2, 4); i > 0; --i) flower(FLOWER_B, false); bladeSet(5, 5, 10, 4.0f, 0.1f, 0.0f); break;
        case PlantKind::CAMPION: for (int i = r.i(2, 4); i > 0; --i) flower(FLOWER_R, false); bladeSet(5, 5, 10, 4.0f, 0.1f, 0.0f); break;
        case PlantKind::CHAMOMILE: for (int i = r.i(2, 4); i > 0; --i) flower(FLOWER_W, true); bladeSet(5, 5, 10, 4.0f, 0.1f, 0.0f); break;
        case PlantKind::NETTLE:
            for (int i = r.i(2, 4); i > 0; --i) {
                const float x = cx + (r.f() - 0.5f) * 8, z = cz + (r.f() - 0.5f) * 8;
                const int h = r.i(12, 20);
                stalk(o, x, z, h, 0, 0, 0, NETTLE);
                for (int y = 3; y < h; y += 3) {   // супротивные листья накрест, пониклые
                    const bool ax = (y / 3) & 1;
                    for (int s = 1; s <= 3; ++s) {
                        const float dy = float(y) - (s == 3 ? 1.0f : 0.0f);
                        put(o, x + (ax ? s : 0), dy, z + (ax ? 0 : s), NETTLE); put(o, x - (ax ? s : 0), dy, z - (ax ? 0 : s), NETTLE);
                    }
                }
            }
            break;
        case PlantKind::FERN: {
            const int n = r.i(5, 8);
            const float a0 = r.f() * 6.28f;
            for (int i = 0; i < n; ++i) {   // вайи: дугой вверх и наружу, по бокам - доли, к концу короче
                const float a = a0 + 6.283f * i / n + (r.f() - 0.5f) * 0.4f;
                const float ux = std::cos(a), uz = std::sin(a);
                const int L = r.i(10, 14);
                for (int s = 0; s < L; ++s) {
                    const float t = float(s) / L;
                    const float x = cx + ux * s, z = cz + uz * s, y = 9.0f * std::sin(t * 2.4f) * (0.7f + 0.3f * r.f() * 0.0f);
                    put(o, x, y, z, FERN);
                    if (s > 1 && (s & 1)) {
                        const float pw = 3.0f * (1.0f - t) + 1.0f;
                        for (int q = 1; q <= int(pw); ++q) { put(o, x - uz * q, y - q * 0.3f, z + ux * q, FERN); put(o, x + uz * q, y - q * 0.3f, z - ux * q, FERN); }
                    }
                }
            }
            break;
        }
        case PlantKind::MOSS:
            for (int z = 8; z < 24; ++z) for (int x = 8; x < 24; ++x) if (mix(uint32_t(v) * 31u ^ uint32_t(x * 31 + z * 977)) % 5u < 3u) put(o, float(x), 0, float(z), MOSS);
            break;
        case PlantKind::REED: {
            for (int i = r.i(5, 9); i > 0; --i) {
                const float x = cx + (r.f() - 0.5f) * 8, z = cz + (r.f() - 0.5f) * 8;
                const int h = r.i(20, 31);
                stalk(o, x, z, h, (r.f() - 0.5f) * 0.1f, (r.f() - 0.5f) * 0.1f, 0.1f, REED);
                if (r.f() < 0.4f) for (int y = h - 6; y < h - 1; ++y) { put(o, x, float(y), z, CATTAIL); put(o, x + 1, float(y), z, CATTAIL); }   // рогоз
            }
            break;
        }
        default: break;
    }
}

}  // namespace world
