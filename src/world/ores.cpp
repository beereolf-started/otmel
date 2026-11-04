#include "world/ores.h"

#include <algorithm>
#include <cmath>

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

constexpr int CELL = 48;        // сторона ячейки рудных тел, блоки
constexpr int LAYER = 5;        // расстояние между плоскостями напластования, мелкие воксели

}  // namespace

const OreDef& oreDef(int ore) { return TABLE[ore < 0 ? 0 : (ore >= ORE_COUNT ? ORE_COUNT - 1 : ore)]; }

int pickOre(Material host, int32_t depth, uint32_t roll) {
    // Зона окисления: наверху сульфиды выветрены, остаются окислы и карбонаты. Правило общее для
    // всех пород, поэтому идёт первым (Р76: поясов глубины нет, есть процессы).
    if (depth < 25) {
        static const Ore OX[4] = {MALACHITE, LIMONITE, HEMATITE, NATIVE_COPPER};
        return OX[roll % 4];
    }
    switch (kindOf(host)) {
        case ACID: {   // порфировые и гидротермальные у кислых глубинных пород
            static const Ore A[8] = {CHALCOPYRITE, GALENA, SPHALERITE, NATIVE_GOLD,
                                     CASSITERITE, NATIVE_SILVER, BISMUTHINITE, TETRAHEDRITE};
            return A[roll % 8];
        }
        case BASIC: {   // магматические - внутри основных пород
            static const Ore B[3] = {MAGNETITE, GARNIERITE, NATIVE_COPPER};
            return B[roll % 3];
        }
        case VOLCANIC: {
            static const Ore V[4] = {NATIVE_COPPER, HEMATITE, SULFUR, CHALCOPYRITE};
            return V[roll % 4];
        }
        case SEDIMENT: {   // в своих пластах: угли, испарения, сера
            static const Ore S[6] = {COAL, LIGNITE, GYPSUM, HALITE, SULFUR, LIMONITE};
            return S[roll % 6];
        }
        default: {
            static const Ore M[4] = {GRAPHITE, SPHALERITE, NATIVE_GOLD, HEMATITE};
            return M[roll % 4];
        }
    }
}

void oreSpecks(uint32_t seed, int32_t bx, int32_t by, int32_t bz, int side, std::vector<OreSpeck>& out) {
    out.clear();
    // Тела ищем в ячейках, задевающих куб, плюс кольцо: тело заводится внутри своей ячейки, но
    // растекается прожилками и может достать до соседней.
    const int32_t c0x = floorDiv(bx, CELL) - 1, c0y = floorDiv(by, CELL) - 1, c0z = floorDiv(bz, CELL) - 1;
    const int32_t c1x = floorDiv(bx + side - 1, CELL), c1y = floorDiv(by + side - 1, CELL), c1z = floorDiv(bz + side - 1, CELL);
    const int32_t f0x = bx * FINE, f0y = by * FINE, f0z = bz * FINE, fside = side * FINE;
    for (int32_t cz = c0z; cz <= c1z; ++cz)
        for (int32_t cy = c0y; cy <= c1y; ++cy)
            for (int32_t cx = c0x; cx <= c1x; ++cx) {
                const uint32_t h = hash3(cx, cy, cz, seed ^ 0x0E5A1u);
                if ((h & 255u) >= 30u) continue;   // тело примерно в одной ячейке из девяти
                // Тело: середина в ячейке, протяжённость вдоль слоёв больше, чем поперёк - рудные
                // залежи вытянуты по напластованию, а не шарообразны.
                const int32_t mx = cx * CELL * FINE + int32_t((h >> 8) % uint32_t(CELL * FINE));
                const int32_t my = cy * CELL * FINE + int32_t((h >> 14) % uint32_t(CELL * FINE));
                const int32_t mz = cz * CELL * FINE + int32_t((h >> 20) % uint32_t(CELL * FINE));
                const int32_t rH = 16 + int32_t((h >> 3) % 56u);    // поперёк слоёв - от блока до четырёх
                const int32_t rV = 6 + int32_t((h >> 26) % 20u);    // вдоль вертикали - тоньше
                const int n = 40 + int((h >> 11) % 220u);           // сколько вкраплений в теле
                for (int k = 0; k < n; ++k) {
                    const uint32_t s = mix(h ^ (uint32_t(k) * 0x9E3779B1u));
                    // ПЛОСКОСТИ НАПЛАСТОВАНИЯ: высота вкрапления прижата к слою, и только внутри
                    // слоя есть малый разброс. Отсюда прожилки, а не равномерный крап.
                    const int lay = int((s >> 2) % uint32_t(2 * rV / LAYER + 1)) - rV / LAYER;
                    const int32_t dy = lay * LAYER + int32_t((s >> 9) % 2u);
                    // по слою - свободно, но к краю тела реже: радиус берём с уклоном к середине
                    const uint32_t a = (s >> 11) % 1024u;
                    const float ang = float(a) * 0.0061359231f;
                    const uint32_t rr = (s >> 21) % 1024u;
                    const float rad = float(rH) * float(rr) / 1024.0f;
                    const int32_t dx = int32_t(rad * std::cos(ang)), dz = int32_t(rad * std::sin(ang));
                    // размер: почти всегда крупинка, изредка самородок в несколько кубов
                    const uint32_t sz = s & 255u;
                    const uint8_t size = sz < 200u ? uint8_t(1 + (sz & 1u)) : (sz < 250u ? 3 : uint8_t(5 + (sz & 3u)));
                    const int32_t x = mx + dx, y = my + dy, z = mz + dz;
                    if (x + size <= f0x || x >= f0x + fside || y + size <= f0y || y >= f0y + fside ||
                        z + size <= f0z || z >= f0z + fside)
                        continue;
                    out.push_back({x, y, z, size, h});
                }
            }
}

}  // namespace world
