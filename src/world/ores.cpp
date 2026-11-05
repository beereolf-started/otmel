#include "world/ores.h"

#include <algorithm>
#include <cmath>

#include "gen/fixed.h"
#include "gen/noise.h"
#include "world/terrain.h"

namespace world {
namespace {

// Порядок неизменен: номер руды идёт в материал и в сохранения.
enum Ore : uint8_t {
    MALACHITE, NATIVE_COPPER, CHALCOPYRITE, TETRAHEDRITE, LIMONITE, HEMATITE, MAGNETITE, CASSITERITE,
    SPHALERITE, GALENA, NATIVE_GOLD, NATIVE_SILVER, BISMUTHINITE, GARNIERITE, COAL, LIGNITE,
    GYPSUM, HALITE, SULFUR, GRAPHITE
};
const OreDef TABLE[ORE_COUNT] = {
    {"малахит"}, {"самородная медь"}, {"халькопирит"}, {"тетраэдрит"}, {"лимонит"},
    {"гематит"}, {"магнетит"}, {"касситерит"}, {"сфалерит"}, {"галенит"},
    {"самородное золото"}, {"самородное серебро"}, {"висмутин"}, {"гарниерит"}, {"каменный уголь"},
    {"бурый уголь"}, {"гипс"}, {"каменная соль"}, {"сера"}, {"графит"},
};

uint32_t mix(uint32_t h) {
    h ^= h >> 15; h *= 0x2C1B3C6Du; h ^= h >> 12; h *= 0x297A2D39u; h ^= h >> 15;
    return h;
}
uint32_t hash3(int32_t x, int32_t y, int32_t z, uint32_t seed) {
    return mix(uint32_t(x) * 0x8DA6B343u ^ uint32_t(y) * 0xD8163841u ^ uint32_t(z) * 0xCB1AB31Fu ^ seed);
}
int32_t floorDiv(int32_t a, int32_t b) { return a >= 0 ? a / b : -(((-a) + b - 1) / b); }

enum Kind : uint8_t { ACID, BASIC, VOLCANIC, SEDIMENT, META };

Kind kindOf(Material host) {
    switch (int(host) - ROCK_FIRST) {
        case GRANITE: case DIORITE: case GRANODIORITE: case MONZONITE: return ACID;
        case GABBRO: case PERIDOTITE: case SERPENTINITE: case AMPHIBOLITE: return BASIC;
        case RHYOLITE: case BASALT: case ANDESITE: case DACITE: case TRACHYTE:
        case LATITE: case BASALTIC_ANDESITE: case OBSIDIAN: return VOLCANIC;
        case SHALE: case CLAYSTONE: case LIMESTONE: case CONGLOMERATE: case DOLOMITE: case CHERT:
        case CHALK: case SANDSTONE: case SILTSTONE: case MARL: case GREYWACKE: return SEDIMENT;
        default: return META;
    }
}

constexpr int VCELL = 96;       // Р89: ячейка жил, блоки: жила - примерно в каждой второй
constexpr uint32_t VEIN_P = 140; // из 256

float frand(uint32_t& h) { h = mix(h + 0x9E3779B9u); return float(h & 0xFFFFFF) / 16777216.0f; }

}  // namespace

const OreDef& oreDef(int ore) { return TABLE[ore < 0 ? 0 : (ore >= ORE_COUNT ? ORE_COUNT - 1 : ore)]; }

int pickOre(Material host, int32_t depth, uint32_t roll) {
    // Р89: у минералов ВЕСА, а не равные доли: золото и серебро - редкость, а не каждая восьмая жила.
    struct W { Ore o; int w; };
    const auto pick = [&](const W* t, int n) {
        int sum = 0;
        for (int i = 0; i < n; ++i) sum += t[i].w;
        int r = int(mix(roll) % uint32_t(sum));
        for (int i = 0; i < n; ++i) { if (r < t[i].w) return int(t[i].o); r -= t[i].w; }
        return int(t[0].o);
    };
    // Зона окисления: наверху сульфиды выветрены, остаются окислы и карбонаты (Р76: процессы, а не пояса)
    if (depth < 25) {
        static const W OX[] = {{MALACHITE, 3}, {LIMONITE, 4}, {HEMATITE, 3}, {NATIVE_COPPER, 2}};
        return pick(OX, 4);
    }
    switch (kindOf(host)) {
        case ACID: {   // порфировые и гидротермальные у кислых глубинных пород
            static const W A[] = {{CHALCOPYRITE, 5}, {GALENA, 4}, {SPHALERITE, 4}, {CASSITERITE, 3},
                                  {TETRAHEDRITE, 3}, {BISMUTHINITE, 2}, {NATIVE_SILVER, 1}, {NATIVE_GOLD, 1}};
            return pick(A, 8);
        }
        case BASIC: {   // магматические - внутри основных пород
            static const W B[] = {{MAGNETITE, 5}, {GARNIERITE, 3}, {CHALCOPYRITE, 2}, {NATIVE_COPPER, 1}};
            return pick(B, 4);
        }
        case VOLCANIC: {
            static const W V[] = {{NATIVE_COPPER, 3}, {HEMATITE, 3}, {SULFUR, 3}, {CHALCOPYRITE, 2}};
            return pick(V, 4);
        }
        case SEDIMENT: {   // в своих пластах: угли, испарения, осадочное железо
            static const W S[] = {{COAL, 4}, {LIGNITE, 3}, {LIMONITE, 3}, {GYPSUM, 2}, {HALITE, 2}, {SULFUR, 1}};
            return pick(S, 6);
        }
        default: {
            static const W M[] = {{GRAPHITE, 4}, {HEMATITE, 3}, {MAGNETITE, 2}, {SPHALERITE, 2}, {NATIVE_GOLD, 1}};
            return pick(M, 5);
        }
    }
}

void veinsNear(uint32_t seed, int32_t bx, int32_t by, int32_t bz, int side, std::vector<Vein>& out) {
    out.clear();
    constexpr float MAX_REACH = 100.0f;   // половина длины + изгиб + сечение - не больше
    const int32_t c0x = floorDiv(bx - int32_t(MAX_REACH), VCELL), c1x = floorDiv(bx + side + int32_t(MAX_REACH), VCELL);
    const int32_t c0y = floorDiv(by - int32_t(MAX_REACH), VCELL), c1y = floorDiv(by + side + int32_t(MAX_REACH), VCELL);
    const int32_t c0z = floorDiv(bz - int32_t(MAX_REACH), VCELL), c1z = floorDiv(bz + side + int32_t(MAX_REACH), VCELL);
    for (int32_t cz = c0z; cz <= c1z; ++cz)
        for (int32_t cy = c0y; cy <= c1y; ++cy)
            for (int32_t cx = c0x; cx <= c1x; ++cx) {
                uint32_t h = hash3(cx, cy, cz, seed ^ 0x7E1A5u);
                if ((h & 255u) >= VEIN_P) continue;
                Vein v{};
                v.seed = h;
                v.c[0] = float(cx * VCELL) + frand(h) * VCELL;
                v.c[1] = float(cy * VCELL) + frand(h) * VCELL;
                v.c[2] = float(cz * VCELL) + frand(h) * VCELL;
                // ось: чаще пологая (вдоль напластования), изредка крутая (секущая жила)
                const float yaw = frand(h) * 6.2831853f;
                const float pitch = (frand(h) - 0.5f) * (frand(h) < 0.75f ? 0.7f : 2.6f);
                v.d[0] = std::cos(pitch) * std::cos(yaw);
                v.d[1] = std::sin(pitch);
                v.d[2] = std::cos(pitch) * std::sin(yaw);
                // поперечные оси: n1 горизонтальна, n2 = d × n1
                float l = std::sqrt(v.d[0] * v.d[0] + v.d[2] * v.d[2]);
                v.n1[0] = -v.d[2] / l; v.n1[1] = 0.0f; v.n1[2] = v.d[0] / l;
                v.n2[0] = v.d[1] * v.n1[2] - v.d[2] * v.n1[1];
                v.n2[1] = v.d[2] * v.n1[0] - v.d[0] * v.n1[2];
                v.n2[2] = v.d[0] * v.n1[1] - v.d[1] * v.n1[0];
                v.half = 20.0f + frand(h) * 45.0f;
                v.R = 3.0f + frand(h) * 6.0f;
                v.flat = 0.45f + frand(h) * 0.55f;
                // изгиб: дуга, середина отходит от хорды до трети половины длины
                const float b1 = (frand(h) - 0.5f) * 0.7f * v.half, b2 = (frand(h) - 0.5f) * 0.4f * v.half;
                for (int i = 0; i < 3; ++i) v.bend[i] = v.n1[i] * b1 + v.n2[i] * b2;
                v.roll = mix(h ^ 0xA11CEu);
                v.roll2 = mix(h ^ 0xB0B0u);
                v.reach = v.half + std::sqrt(b1 * b1 + b2 * b2) + v.R * 1.5f;
                // шар жилы задевает куб?
                float dist2 = 0.0f;
                const float lo[3] = {float(bx), float(by), float(bz)};
                for (int i = 0; i < 3; ++i) {
                    const float q = std::clamp(v.c[i], lo[i], lo[i] + float(side));
                    dist2 += (v.c[i] - q) * (v.c[i] - q);
                }
                if (dist2 <= v.reach * v.reach) out.push_back(v);
            }
}

float veinRichness(const Vein& v, int32_t x, int32_t y, int32_t z) {
    const float p[3] = {float(x) + 0.5f - v.c[0], float(y) + 0.5f - v.c[1], float(z) + 0.5f - v.c[2]};
    const float s = p[0] * v.d[0] + p[1] * v.d[1] + p[2] * v.d[2];
    if (std::abs(s) > v.half) return 0.0f;
    const float t = s / v.half, arc = 1.0f - t * t;
    // сечение сужается к концам жилы
    const float r = v.R * std::pow(arc, 0.35f);
    float q[3];
    for (int i = 0; i < 3; ++i) q[i] = p[i] - v.d[i] * s - v.bend[i] * arc;
    const float u = q[0] * v.n1[0] + q[1] * v.n1[1] + q[2] * v.n1[2];
    const float w = (q[0] * v.n2[0] + q[1] * v.n2[1] + q[2] * v.n2[2]) / v.flat;
    const float e0 = (u * u + w * w) / (r * r * 2.0f);
    if (e0 > 1.0f) return 0.0f;   // заведомо вне: шум сдвигает край не больше чем на 0.4
    const float e = std::sqrt(u * u + w * w) / r;
    // рваный край: трёхмерный шум ~8 блоков
    const float n = float(gen::simplex3(x * 16, y * 16, z * 16, 7, v.seed)) / float(fx::ONE);
    return std::clamp(1.0f - e + 0.4f * n, 0.0f, 1.0f);
}

}  // namespace world
