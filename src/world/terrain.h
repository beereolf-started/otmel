#pragma once
// Рельеф по карте высот (Р17). Пока - только равнины на Перлине (Р15); смесь по биомам - следующий шаг.
#include <cstdint>

#include "world/chunk.h"

namespace world {

struct TerrainParams {
    uint32_t seed = 1337;
    int32_t baseHeight = 32 * VOXELS_PER_BLOCK;   // средняя высота: 32 блока, в вокселях
    int32_t amplitude = 20 * VOXELS_PER_BLOCK;    // размах холмов: ±20 блоков
    int firstCellLog2 = 11;                        // первая октава - 2048 вокселей = 128 блоков
    int octaves = 6;                               // мельчайшая - 64 вокселя = 4 блока
};

// Высоты столбцов куска С КАЙМОЙ в один воксель: [-1..32] по x и z, 34×34. Кайма нужна сетке: грань на краю
// куска видна, только если сосед за краем - воздух, а соседа считаем той же формулой, не трогая чужой кусок.
struct ColumnHeights {
    int32_t h[CHUNK + 2][CHUNK + 2];   // [z+1][x+1], высота поверхности в вокселях (верхний твёрдый воксель)
    int32_t minH, maxH;
};

void columnHeights(const TerrainParams& p, int32_t chunkX, int32_t chunkZ, ColumnHeights& out);

// Заполнить кусок по высотам: камень, 3 блока земли, сверху воксель травы
void fillChunk(const ColumnHeights& heights, Chunk& c);

// Контрольная сумма вокселей куска - для проверки воспроизводимости (Р2)
uint64_t checksum(const Chunk& c);

}  // namespace world
