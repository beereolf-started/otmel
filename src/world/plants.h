#pragma once
// Р107: ТРАВЫ И МЕЛКАЯ ПОРОСЛЬ - мелкими вокселями (1/16 блока) на верху травяных блоков вокруг игрока.
//
// Что растёт на блоке - решает место: сообщество по шуму пятен и по соседству - луг (трава, клевер, цветы),
// лес (папоротник, мох, крапива), у воды (камыш с рогозом). Само растение и его форма - жребий блока:
// одна и та же точка мира - всегда то же растение.
#include <cstdint>
#include <vector>

#include "world/chunk.h"
#include "world/terrain.h"

namespace world {

enum class PlantKind : uint8_t { NONE, GRASS, TALL_GRASS, CLOVER, DANDELION, BLUEBELL, CAMPION, CHAMOMILE, NETTLE, FERN, MOSS, REED, COUNT };
const char* plantName(PlantKind k);

/** Растение на блоке (x, z) поверх верхнего вокселя. forest - густота леса 0..1, wetNear - вода в паре блоков. */
PlantKind plantAt(const TerrainParams& p, int32_t x, int32_t z, Material top, float forest, bool wetNear);

struct PlantVoxel { int8_t x, y, z; Material m; };   // от угла блока над верхом, мелкие воксели (x, z 0..15, y 0..)
/** Воксели растения: жребий h - форма. */
void stampPlant(PlantKind k, uint32_t h, std::vector<PlantVoxel>& out);

}  // namespace world
