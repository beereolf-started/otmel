#pragma once
// Р107-Р109: ТРАВЫ И МЕЛКАЯ ПОРОСЛЬ.
//
// Р109: растение - не воксели на блоке, а КОПИЯ ГОТОВОЙ МОДЕЛИ: у каждого вида несколько вариантов модели
// (мелкие воксели 1/16 блока, корень в середине куска 32³), а в мире - тысячи копий в произвольных точках
// поверхности со своим поворотом и размером. Видеокарта рисует все копии одной модели одним вызовом, поэтому
// траву можно ставить густо и далеко. Луговая трава - по пояс.
//
// Что растёт - решает место: сообщество по пятнам шума и соседству - луг (трава, клевер, цветы), лес
// (низкая трава, папоротник, мох, крапива), у воды (камыш с рогозом). Жребий - точка мира: всегда то же.
#include <cstdint>
#include <vector>

#include "world/chunk.h"
#include "world/terrain.h"

namespace world {

enum class PlantKind : uint8_t { NONE, GRASS, TALL_GRASS, CLOVER, DANDELION, BLUEBELL, CAMPION, CHAMOMILE, NETTLE, FERN, MOSS, REED,
                                 SHORT_GRASS, COUNT };
constexpr int PLANT_VARIANTS = 6;
const char* plantName(PlantKind k);

/** Растение пучка salt на блоке (x, z). forest - густота леса 0..1, wetNear - вода рядом. */
PlantKind plantAt(const TerrainParams& p, int32_t x, int32_t z, Material top, float forest, bool wetNear, uint32_t salt = 0);

struct PlantVoxel { int8_t x, y, z; Material m; };   // мелкие воксели куска 32³, корень - (16, 0, 16)
/** Воксели модели вида k, вариант v. */
void stampPlant(PlantKind k, int v, std::vector<PlantVoxel>& out);

}  // namespace world
