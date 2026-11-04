#include "world/soil_table.h"

namespace world {
namespace {

// Порядок неизменен: номер почвы идёт в материал. Доли - по учебному треугольнику почв, огрублённо.
const SoilDef TABLE[SOIL_COUNT] = {
    {"песок",               {0.90f, 0.06f, 0.02f, 0.02f}, "tfc:block/sand/yellow"},
    {"супесь",              {0.70f, 0.18f, 0.08f, 0.04f}, "tfc:block/dirt/sandy_loam"},
    {"суглинок",            {0.42f, 0.38f, 0.16f, 0.04f}, "tfc:block/dirt/loam"},
    {"пылеватый суглинок",  {0.22f, 0.58f, 0.16f, 0.04f}, "tfc:block/dirt/silty_loam"},
    {"пыль",                {0.10f, 0.80f, 0.07f, 0.03f}, "tfc:block/dirt/silt"},
    {"глинистый суглинок",  {0.28f, 0.34f, 0.34f, 0.04f}, "tfc:block/clay/loam"},
    {"глина",               {0.14f, 0.24f, 0.60f, 0.02f}, "tfc:block/clay/silty_loam"},
    {"ил",                  {0.16f, 0.46f, 0.26f, 0.12f}, "tfc:block/mud/silt"},
    {"торф",                {0.06f, 0.16f, 0.08f, 0.70f}, "tfc:block/peat"},
};

}  // namespace

const SoilDef& soilDef(int soil) { return TABLE[soil < 0 ? 0 : (soil >= SOIL_COUNT ? SOIL_COUNT - 1 : soil)]; }

int nearestSoil(const SoilComp& c) {
    // Перегной весит больше прочего: торф от суглинка отличается именно им, а доли песка и пыли у
    // них близки. Без веса торф никогда бы не выпал.
    int best = 0;
    float bestD = 1e30f;
    for (int i = 0; i < SOIL_COUNT; ++i) {
        const SoilComp& t = TABLE[i].comp;
        const float ds = c.sand - t.sand, di = c.silt - t.silt, dc = c.clay - t.clay, dg = c.organic - t.organic;
        const float d = ds * ds + di * di + dc * dc + 3.0f * dg * dg;
        if (d < bestD) { bestD = d; best = i; }
    }
    return best;
}

}  // namespace world
