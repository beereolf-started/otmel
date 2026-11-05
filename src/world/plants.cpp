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
void put(std::vector<PlantVoxel>& o, int x, int y, int z, Material m) {
    if (x < 0 || x > 15 || z < 0 || z > 15 || y < 0 || y > 40) return;
    o.push_back({int8_t(x), int8_t(y), int8_t(z), m});
}
// стебель от (x, z) вверх на h, с наклоном (dx, dz) на каждые два вокселя
void stem(std::vector<PlantVoxel>& o, int x, int z, int h, int dx, int dz, Material m, int y0 = 0) {
    for (int y = 0; y < h; ++y) put(o, x + dx * (y / 2), y0 + y, z + dz * (y / 2), m);
}

}  // namespace

const char* plantName(PlantKind k) {
    static const char* N[] = {"", "трава", "высокая трава", "клевер", "одуванчик", "колокольчик", "смолёвка", "ромашка",
                              "крапива", "папоротник", "мох", "камыш"};
    return N[int(k)];
}

PlantKind plantAt(const TerrainParams& p, int32_t x, int32_t z, Material top, float forest, bool wetNear) {
    uint32_t h = mix(uint32_t(x) * 0x8DA6B343u ^ uint32_t(z) * 0xD8163841u ^ p.seed * 0x9E3779B1u ^ 0x91A47u);
    const float u = float(h & 0xFFFF) / 65536.0f;
    if (wetNear) return u < 0.45f ? PlantKind::REED : (u < 0.7f && isGrass(top) ? PlantKind::TALL_GRASS : PlantKind::NONE);
    if (!isGrass(top)) return PlantKind::NONE;
    const float patch = float(gen::value2(x, z, 4, p.seed ^ 0x7A7C4u)) / 65536.0f;   // пятна по 16 блоков
    const float patch2 = float(gen::value2(x, z, 3, p.seed ^ 0x1CE5u)) / 65536.0f;
    if (forest > 0.25f) {   // подлесок: папоротник, мох, крапива пятнами
        if (u < 0.16f) return PlantKind::FERN;
        if (u < 0.30f) return PlantKind::MOSS;
        if (u < 0.36f && patch > 0.2f) return PlantKind::NETTLE;
        if (u < 0.52f) return PlantKind::GRASS;
        return PlantKind::NONE;
    }
    // луг
    if (u < 0.40f) return PlantKind::GRASS;
    if (u < 0.52f) return patch > -0.1f ? PlantKind::TALL_GRASS : PlantKind::GRASS;
    if (u < 0.60f) return patch < -0.15f ? PlantKind::CLOVER : PlantKind::GRASS;
    if (u < 0.66f) {   // цветы пятнами: где одуванчики, где колокольчики
        if (patch2 > 0.3f) return PlantKind::DANDELION;
        if (patch2 > 0.0f) return PlantKind::CHAMOMILE;
        if (patch2 > -0.3f) return PlantKind::CAMPION;
        return PlantKind::BLUEBELL;
    }
    if (u < 0.68f && patch > 0.4f) return PlantKind::NETTLE;
    return PlantKind::NONE;
}

void stampPlant(PlantKind k, uint32_t seed, std::vector<PlantVoxel>& o) {
    o.clear();
    R r{mix(seed)};
    const auto blade = [&](int hmin, int hmax, int spread) {
        const int x = r.i(2, 13), z = r.i(2, 13);
        const int dx = r.i(-spread, spread), dz = r.i(-spread, spread);
        stem(o, x, z, r.i(hmin, hmax), dx, dz, r.f() < 0.5f ? GRASS_BLADE : GRASS_BLADE2);
    };
    const auto flower = [&](Material head, bool ring) {
        const int x = r.i(4, 11), z = r.i(4, 11), h = r.i(5, 9);
        stem(o, x, z, h, 0, 0, GRASS_BLADE);
        if (ring) {   // ромашка: белые лепестки кругом, жёлтая середина
            for (int a = -1; a <= 1; ++a) for (int b = -1; b <= 1; ++b) put(o, x + a, h, z + b, (a || b) ? FLOWER_W : FLOWER_Y);
        } else {
            put(o, x, h, z, head); put(o, x + 1, h, z, head); put(o, x, h, z + 1, head); put(o, x, h + 1, z, head);
        }
        put(o, x + 1, 0, z, GRASS_BLADE); put(o, x - 1, 0, z, GRASS_BLADE); put(o, x, 1, z + 1, GRASS_BLADE2);   // прикорневые листья
    };
    switch (k) {
        case PlantKind::GRASS: { const int n = r.i(4, 7); for (int i = 0; i < n; ++i) blade(2, 5, 1); break; }
        case PlantKind::TALL_GRASS: {
            const int n = r.i(6, 10);
            for (int i = 0; i < n; ++i) blade(6, 11, 1);
            for (int i = 0; i < 2; ++i) { const int x = r.i(3, 12), z = r.i(3, 12), h = r.i(9, 13); stem(o, x, z, h, 0, 0, GRASS_BLADE2); put(o, x, h, z, REED); put(o, x, h + 1, z, REED); }
            break;
        }
        case PlantKind::CLOVER: {
            const int n = r.i(6, 10);
            for (int i = 0; i < n; ++i) {   // тройчатый лист низко
                const int x = r.i(1, 14), z = r.i(1, 14), y = r.i(0, 1);
                put(o, x, y, z, CLOVER); put(o, x + 1, y, z, CLOVER); put(o, x, y, z + 1, CLOVER); put(o, x - 1, y, z, CLOVER);
            }
            for (int i = r.i(1, 2); i > 0; --i) { const int x = r.i(3, 12), z = r.i(3, 12); stem(o, x, z, 3, 0, 0, CLOVER); put(o, x, 3, z, r.f() < 0.5f ? FLOWER_W : FLOWER_R); }
            break;
        }
        case PlantKind::DANDELION: for (int i = r.i(1, 3); i > 0; --i) flower(FLOWER_Y, false); break;
        case PlantKind::BLUEBELL: for (int i = r.i(1, 3); i > 0; --i) flower(FLOWER_B, false); break;
        case PlantKind::CAMPION: for (int i = r.i(1, 3); i > 0; --i) flower(FLOWER_R, false); break;
        case PlantKind::CHAMOMILE: for (int i = r.i(1, 3); i > 0; --i) flower(FLOWER_W, true); break;
        case PlantKind::NETTLE:
            for (int i = r.i(1, 3); i > 0; --i) {
                const int x = r.i(4, 11), z = r.i(4, 11), h = r.i(8, 14);
                stem(o, x, z, h, 0, 0, NETTLE);
                for (int y = 2; y < h; y += 3) {   // супротивные листья, пары накрест
                    const bool ax = (y / 3) & 1;
                    put(o, x + (ax ? 1 : 0), y, z + (ax ? 0 : 1), NETTLE); put(o, x - (ax ? 1 : 0), y, z - (ax ? 0 : 1), NETTLE);
                    put(o, x + (ax ? 2 : 0), y - 1, z + (ax ? 0 : 2), NETTLE); put(o, x - (ax ? 2 : 0), y - 1, z - (ax ? 0 : 2), NETTLE);
                }
            }
            break;
        case PlantKind::FERN: {
            const int n = r.i(4, 6);
            const float a0 = r.f() * 6.28f;
            for (int i = 0; i < n; ++i) {   // вайи: дугой вверх и наружу, по бокам - доли
                const float a = a0 + 6.283f * i / n;
                const float cx = std::cos(a), cz = std::sin(a);
                const int L = r.i(6, 9);
                for (int s = 0; s < L; ++s) {
                    const float t = float(s) / L;
                    const int x = 8 + int(std::lround(cx * s)), z = 8 + int(std::lround(cz * s));
                    const int y = int(std::lround(4.5f * std::sin(t * 2.6f)));
                    put(o, x, y, z, FERN);
                    if (s > 1 && (s & 1)) { put(o, x + int(std::lround(-cz)), y, z + int(std::lround(cx)), FERN); put(o, x - int(std::lround(-cz)), y, z - int(std::lround(cx)), FERN); }
                }
            }
            break;
        }
        case PlantKind::MOSS:
            for (int z = 0; z < 16; ++z) for (int x = 0; x < 16; ++x) if (mix(seed ^ uint32_t(x * 31 + z * 977)) % 5u < 2u) put(o, x, 0, z, MOSS);
            break;
        case PlantKind::REED: {
            const int n = r.i(3, 6);
            for (int i = 0; i < n; ++i) {
                const int x = r.i(3, 12), z = r.i(3, 12), h = r.i(18, 30);
                stem(o, x, z, h, 0, 0, REED);
                if (i == 0) for (int y = h - 5; y < h - 1; ++y) { put(o, x, y, z, CATTAIL); put(o, x + 1, y, z, CATTAIL); }   // рогоз
            }
            break;
        }
        default: break;
    }
}

}  // namespace world
