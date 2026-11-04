#include "world/ores.h"

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
    {"малахит", "tfc:block/ore/normal_malachite"},
    {"самородная медь", "tfc:block/ore/normal_native_copper"},
    {"халькопирит", "tfc:block/ore/pyrite"},
    {"тетраэдрит", "tfc:block/ore/normal_tetrahedrite"},
    {"лимонит", "tfc:block/ore/normal_limonite"},
    {"гематит", "tfc:block/ore/normal_hematite"},
    {"магнетит", "tfc:block/ore/normal_magnetite"},
    {"касситерит", "tfc:block/ore/normal_cassiterite"},
    {"сфалерит", "tfc:block/ore/normal_sphalerite"},
    {"галенит", "tfc:block/ore/normal_bismuthinite"},
    {"самородное золото", "tfc:block/ore/normal_native_gold"},
    {"самородное серебро", "tfc:block/ore/normal_native_silver"},
    {"висмутин", "tfc:block/ore/rich_bismuthinite"},
    {"гарниерит", "tfc:block/ore/normal_garnierite"},
    {"каменный уголь", "tfc:block/ore/bituminous_coal"},
    {"бурый уголь", "tfc:block/ore/lignite"},
    {"гипс", "tfc:block/ore/gypsum"},
    {"каменная соль", "tfc:block/ore/halite"},
    {"сера", "tfc:block/ore/sulfur"},
    {"графит", "tfc:block/ore/graphite"},
};

uint32_t hash3(int32_t x, int32_t y, int32_t z, uint32_t seed) {
    uint32_t h = uint32_t(x) * 0x8DA6B343u ^ uint32_t(y) * 0xD8163841u ^ uint32_t(z) * 0xCB1AB31Fu ^ seed;
    h ^= h >> 15; h *= 0x2C1B3C6Du; h ^= h >> 12; h *= 0x297A2D39u; h ^= h >> 15;
    return h;
}

int32_t floorDiv(int32_t a, int32_t b) { return a >= 0 ? a / b : -(((-a) + b - 1) / b); }

enum Kind : uint8_t { ACID, BASIC, VOLCANIC, SEDIMENT, META };

/** К какому семейству относим породу-хозяина: от него зависит, какие процессы тут работали. */
Kind kindOf(Material host) {
    const int r = int(host) - ROCK_FIRST;
    switch (r) {
        case GRANITE: case DIORITE: case GRANODIORITE: case MONZONITE: return ACID;
        case GABBRO: case PERIDOTITE: case SERPENTINITE: case AMPHIBOLITE: return BASIC;
        case RHYOLITE: case BASALT: case ANDESITE: case DACITE: case TRACHYTE:
        case LATITE: case BASALTIC_ANDESITE: case OBSIDIAN: return VOLCANIC;
        case SHALE: case CLAYSTONE: case LIMESTONE: case CONGLOMERATE: case DOLOMITE: case CHERT:
        case CHALK: case SANDSTONE: case SILTSTONE: case MARL: case GREYWACKE: return SEDIMENT;
        default: return META;
    }
}

/** Какой минерал даёт этот процесс. r - жребий тела, depth - глубина под поверхностью, блоки. */
Ore pick(Kind k, int32_t depth, uint32_t r) {
    // Зона окисления: наверху сульфиды выветрены, и руда здесь - окислы и карбонаты. Это общее
    // правило для всех пород, поэтому оно идёт первым.
    if (depth < 25) {
        static const Ore OX[4] = {MALACHITE, LIMONITE, HEMATITE, NATIVE_COPPER};
        return OX[r % 4];
    }
    switch (k) {
        case ACID: {   // порфировые и гидротермальные у кислых глубинных пород
            static const Ore A[8] = {CHALCOPYRITE, GALENA, SPHALERITE, NATIVE_GOLD,
                                     CASSITERITE, NATIVE_SILVER, BISMUTHINITE, TETRAHEDRITE};
            return A[r % 8];
        }
        case BASIC: {   // магматические - внутри основных пород
            static const Ore B[3] = {MAGNETITE, GARNIERITE, NATIVE_COPPER};
            return B[r % 3];
        }
        case VOLCANIC: {
            static const Ore V[4] = {NATIVE_COPPER, HEMATITE, SULFUR, CHALCOPYRITE};
            return V[r % 4];
        }
        case SEDIMENT: {   // в своих пластах: угли, испарения, сера
            static const Ore S[6] = {COAL, LIGNITE, GYPSUM, HALITE, SULFUR, LIMONITE};
            return S[r % 6];
        }
        default: {
            static const Ore M[4] = {GRAPHITE, SPHALERITE, NATIVE_GOLD, HEMATITE};
            return M[r % 4];
        }
    }
}

constexpr int CELL = 48;   // сторона ячейки рудных тел, блоки

}  // namespace

const OreDef& oreDef(int ore) { return TABLE[ore < 0 ? 0 : (ore >= ORE_COUNT ? ORE_COUNT - 1 : ore)]; }

Material oreAt(uint32_t seed, int32_t x, int32_t y, int32_t z, Material host, int32_t surface) {
    if (host < ROCK_FIRST || host > ROCK_LAST) return AIR;   // руда сидит в породе, не в земле
    const int32_t cx = floorDiv(x, CELL), cy = floorDiv(y, CELL), cz = floorDiv(z, CELL);
    const uint32_t h = hash3(cx, cy, cz, seed ^ 0x0E5A1u);
    if ((h & 255u) >= 28u) return AIR;   // тело есть примерно в девяти ячейках из ста
    // Середина тела - внутри ячейки с отступом: тело не выходит за её край, и соседние ячейки
    // считать не нужно. Это и есть вся цена: один хеш на воксель породы.
    const int32_t r = 2 + int32_t((h >> 8) % 8u);                 // радиус 2..9 блоков - разброс богатства
    const int32_t ox = CELL / 2 + int32_t((h >> 11) % 17u) - 8;
    const int32_t oy = CELL / 2 + int32_t((h >> 16) % 17u) - 8;
    const int32_t oz = CELL / 2 + int32_t((h >> 21) % 17u) - 8;
    const int32_t dx = x - (cx * CELL + ox), dy = y - (cy * CELL + oy), dz = z - (cz * CELL + oz);
    if (dx * dx + dy * dy + dz * dz > r * r) return AIR;
    // Вкрапления, а не монолит: внутри тела рудным становится не каждый воксель. К середине тела
    // густо, к краю редко - иначе у залежи была бы резкая граница шаром.
    const int32_t d2 = dx * dx + dy * dy + dz * dz;
    const uint32_t dens = 40u + 150u * uint32_t(r * r - d2) / uint32_t(r * r);
    if ((hash3(x, y, z, seed ^ 0x0B1D3u) & 255u) >= dens) return AIR;
    const int32_t depth = surface - y;   // воксель = блок (Р54)
    return Material(ORE_FIRST + pick(kindOf(host), depth, h >> 26));
}

}  // namespace world
