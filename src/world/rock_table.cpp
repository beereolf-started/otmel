// Р78: справочник пород - состав и приближение ближайшим именем. Порядок строк здесь совпадает с
// перечнем world::Rock: первые ROCK_TFG - породы TFC/TFG (номера неизменны, попадают в материал),
// дальше промежуточные, закрывающие разрывы между ними.
//
// Доли минералов в каждой строке дают единицу. Зернистость: 0 - стекло и афанит (излившиеся),
// 1 - крупнокристаллическая (глубинные). Сланцеватость: 0 - массивная, 1 - расслоённая в плитку.
#include "world/rock_table.h"

#include <cmath>

#include "world/terrain.h"

namespace world {
namespace {

// краткая запись состава: кварц, шпат, слюда, тёмноцветные, кальцит, доломит, глина, органика
constexpr Composition C(float q, float f, float mi, float ma, float ca, float dol, float cl, float org,
                        float grain, float fabric) {
    return Composition{{q, f, mi, ma, ca, dol, cl, org}, grain, fabric};
}

const RockDef TABLE[] = {
    // ── породы TFC/TFG: порядок неизменен ──────────────────────────────────────
    {"гранит",                 F_DEEP, C(0.28f, 0.62f, 0.07f, 0.03f, 0, 0, 0, 0,             0.85f, 0.00f)},
    {"диорит",                 F_DEEP, C(0.05f, 0.60f, 0.10f, 0.25f, 0, 0, 0, 0,             0.80f, 0.00f)},
    {"габбро",                 F_DEEP, C(0.02f, 0.45f, 0.03f, 0.50f, 0, 0, 0, 0,             0.80f, 0.00f)},
    {"риолит",                 F_LAVA, C(0.28f, 0.62f, 0.07f, 0.03f, 0, 0, 0, 0,             0.10f, 0.00f)},
    {"базальт",                F_LAVA, C(0.02f, 0.45f, 0.03f, 0.50f, 0, 0, 0, 0,             0.05f, 0.00f)},
    {"андезит",                F_LAVA, C(0.05f, 0.60f, 0.10f, 0.25f, 0, 0, 0, 0,             0.10f, 0.00f)},
    {"дацит",                  F_LAVA, C(0.20f, 0.58f, 0.08f, 0.14f, 0, 0, 0, 0,             0.12f, 0.00f)},
    {"аргиллитовый сланец",    F_SEDIMENT, C(0.25f, 0, 0.10f, 0, 0, 0, 0.62f, 0.03f,             0.05f, 0.45f)},
    {"аргиллит",               F_SEDIMENT, C(0.18f, 0, 0.05f, 0, 0, 0, 0.75f, 0.02f,             0.03f, 0.10f)},
    {"известняк",              F_SEDIMENT, C(0.03f, 0, 0, 0, 0.92f, 0, 0.05f, 0,                 0.30f, 0.00f)},
    {"конгломерат",            F_SEDIMENT, C(0.55f, 0.20f, 0, 0.10f, 0, 0, 0.15f, 0,             0.90f, 0.00f)},
    {"доломит",                F_SEDIMENT, C(0.02f, 0, 0, 0, 0.07f, 0.88f, 0.03f, 0,             0.35f, 0.00f)},
    {"кремень",                F_SEDIMENT, C(0.96f, 0, 0, 0, 0, 0, 0.03f, 0.01f,                 0.02f, 0.00f)},
    {"мел",                    F_SEDIMENT, C(0, 0, 0, 0, 0.96f, 0, 0.03f, 0.01f,                 0.05f, 0.00f)},
    {"кварцит",                F_META, C(0.95f, 0.03f, 0.02f, 0, 0, 0, 0, 0,                 0.60f, 0.05f)},
    {"кровельный сланец",      F_META, C(0.28f, 0, 0.15f, 0, 0, 0, 0.55f, 0.02f,             0.08f, 0.85f)},
    {"филлит",                 F_META, C(0.32f, 0.03f, 0.40f, 0, 0, 0, 0.25f, 0,             0.20f, 0.80f)},
    {"кристаллический сланец",  F_META, C(0.30f, 0.10f, 0.45f, 0.15f, 0, 0, 0, 0,            0.50f, 0.90f)},
    {"гнейс",                  F_META, C(0.33f, 0.45f, 0.12f, 0.10f, 0, 0, 0, 0,             0.70f, 0.60f)},
    {"мрамор",                 F_META, C(0.03f, 0, 0, 0, 0.95f, 0.02f, 0, 0,                 0.55f, 0.10f)},
    // ── промежуточные (Р78): каждая закрывает разрыв между соседними выше ──────
    {"гранодиорит",            F_DEEP, C(0.20f, 0.62f, 0.09f, 0.09f, 0, 0, 0, 0,             0.83f, 0.00f)},
    {"монцонит",               F_DEEP, C(0.06f, 0.70f, 0.08f, 0.16f, 0, 0, 0, 0,             0.80f, 0.00f)},
    {"перидотит",              F_DEEP, C(0, 0.05f, 0.02f, 0.93f, 0, 0, 0, 0,                 0.75f, 0.00f)},
    {"трахит",                 F_LAVA, C(0.06f, 0.80f, 0.06f, 0.08f, 0, 0, 0, 0,             0.10f, 0.00f)},
    {"латит",                  F_LAVA, C(0.05f, 0.72f, 0.07f, 0.16f, 0, 0, 0, 0,             0.08f, 0.00f)},
    {"базальтовый андезит",    F_LAVA, C(0.03f, 0.52f, 0.05f, 0.40f, 0, 0, 0, 0,             0.07f, 0.00f)},
    {"обсидиан",               F_LAVA, C(0.30f, 0.62f, 0.05f, 0.03f, 0, 0, 0, 0,             0.00f, 0.00f)},
    {"песчаник",               F_SEDIMENT, C(0.78f, 0.12f, 0.03f, 0, 0, 0, 0.07f, 0,             0.55f, 0.00f)},
    {"алевролит",              F_SEDIMENT, C(0.45f, 0, 0.12f, 0, 0, 0, 0.40f, 0.03f,             0.20f, 0.15f)},
    {"мергель",                F_SEDIMENT, C(0.08f, 0, 0, 0, 0.50f, 0, 0.40f, 0.02f,             0.15f, 0.10f)},
    {"граувакка",              F_SEDIMENT, C(0.45f, 0.25f, 0, 0.12f, 0, 0, 0.18f, 0,             0.60f, 0.00f)},
    {"роговик",                F_META, C(0.40f, 0.30f, 0.15f, 0.15f, 0, 0, 0, 0,             0.25f, 0.05f)},
    {"амфиболит",              F_META, C(0.05f, 0.30f, 0.05f, 0.60f, 0, 0, 0, 0,             0.55f, 0.50f)},
    {"мигматит",               F_META, C(0.32f, 0.42f, 0.14f, 0.12f, 0, 0, 0, 0,             0.75f, 0.45f)},
    {"зелёный сланец",         F_META, C(0.18f, 0, 0.25f, 0.45f, 0, 0, 0.12f, 0,             0.30f, 0.85f)},
    {"серпентинит",            F_META, C(0.05f, 0, 0.05f, 0.85f, 0, 0, 0.05f, 0,             0.40f, 0.30f)},
};
static_assert(sizeof(TABLE) / sizeof(TABLE[0]) == ROCK_COUNT, "справочник и перечень Rock разошлись");

// Сложение весит меньше минералов: гранит и риолит различаются только им, но это РАЗНЫЕ породы,
// поэтому вес не нулевой - иначе приближение путало бы глубинные с излившимися.
constexpr float W_GRAIN = 1.6f, W_FABRIC = 1.2f;

}  // namespace

const RockDef& rockDef(int rock) {
    return TABLE[rock < 0 ? 0 : (rock >= ROCK_COUNT ? ROCK_COUNT - 1 : rock)];
}

int nearestRock(const Composition& c, uint8_t families) {
    int best = -1;
    float bestD = 1e30f;
    for (int r = 0; r < ROCK_COUNT; ++r) {
        if (!(TABLE[r].family & families)) continue;
        const Composition& t = TABLE[r].comp;
        float d = 0;
        for (int i = 0; i < MINERALS; ++i) {
            const float e = c.m[i] - t.m[i];
            d += e * e;
        }
        const float g = (c.grain - t.grain) * W_GRAIN, f = (c.fabric - t.fabric) * W_FABRIC;
        d += g * g + f * f;
        if (d < bestD) { bestD = d; best = r; }
    }
    return best < 0 ? nearestRock(c, 0xFF) : best;   // семейство пустое - ищем среди всех
}

Composition mixComp(const Composition& a, const Composition& b, float w) {
    Composition out{};
    for (int i = 0; i < MINERALS; ++i) out.m[i] = a.m[i] + (b.m[i] - a.m[i]) * w;
    out.grain = a.grain + (b.grain - a.grain) * w;
    out.fabric = a.fabric + (b.fabric - a.fabric) * w;
    return out;
}

}  // namespace world
