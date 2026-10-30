#pragma once
// Р65: лес - часть генерации, одна формула на всех расстояниях (Р62): вблизи - дубы уровнями детализации (Р63),
// дальше - спрайты, ещё дальше - шум крон на земле дальних плиток. Все три берут места, семена и густоту отсюда.
//   - ячейки по FOREST_CELL блоков; хеш ячейки (и зерна мира) даёт кандидата - блок в ячейке, сдвиг в мелких
//     вокселях (0..15, не центр блока), семя дуба и бросок 0..1;
//   - густота - гладкая ступень дождей (0.05..0.5 доли) × шум рощ (ячейка 512 блоков: рощи, перелески, поляны) ×
//     0.85; в снегу - 0. Дерево стоит, если бросок < густоты (и место годится: не вода, не круто - проверяет тот,
//     кто знает рельеф).
#include <cstdint>

#include "world/terrain.h"

namespace world {

constexpr int FOREST_CELL = 10;   // блоков

struct TreeCandidate {
    int32_t x, z;      // блок
    int fx, fz;        // сдвиг ствола в мелких вокселях внутри блока
    uint32_t seed;     // семя дуба
    float roll;        // бросок 0..1 (дерево - если меньше густоты)
};
TreeCandidate forestCandidate(const TerrainParams& p, int32_t ci, int32_t cj);

// густота леса 0..1 в точке (блоки) с высотой земли h (воксели = блоки, Р54)
float forestDensity(const TerrainParams& p, int32_t x, int32_t z, int32_t h);
float forestDensity(const TerrainParams& p, int32_t x, int32_t z, int32_t h, const Climate& cl);

}  // namespace world
