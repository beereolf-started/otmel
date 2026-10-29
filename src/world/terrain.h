#pragma once
// Рельеф по карте высот (Р17) и биомы (Р19): форма рельефа и климат - раздельно.
// Форма: три вида высоты (равнины - Перлин, холмы - симплекс, горы - хребты), смешанные непрерывными весами
// по медленному шуму «рельефности» - без ступеньки на границе по построению.
// Климат (по образцу TFC): температура - пояса по z и падение с высотой, осадки - пояса по x; он задаёт покров.
#include <cstdint>

#include "gen/fixed.h"
#include "world/chunk.h"

namespace world {

constexpr fx::q16 DEG = fx::ONE;   // градус Цельсия и миллиметр осадков - в Q16

struct TerrainParams {
    uint32_t seed = 1337;
    // равнины (Р15, Р17) - прежняя формула без изменений
    int32_t baseHeight = 32 * VOXELS_PER_BLOCK;   // средняя высота: 32 блока, в вокселях
    int32_t amplitude = 20 * VOXELS_PER_BLOCK;    // размах: ±20 блоков
    int firstCellLog2 = 11;                        // первая октава - 2048 вокселей = 128 блоков
    int octaves = 6;                               // мельчайшая - 64 вокселя = 4 блока
    // холмы - симплекс: 44 ± 36 блоков, крупнейшая волна 256 блоков, мельчайшая 8
    int32_t hillsBase = 44 * VOXELS_PER_BLOCK, hillsAmp = 36 * VOXELS_PER_BLOCK;
    int hillsCellLog2 = 12, hillsOctaves = 6;
    // горы - хребты: от 34 блоков (дно долин) до 234 (гребни), крупнейший хребет 512 блоков, мельчайший 8
    int32_t mountBase = 34 * VOXELS_PER_BLOCK, mountAmp = 200 * VOXELS_PER_BLOCK;
    int mountCellLog2 = 13, mountOctaves = 7;
    // рельефность - маска на шуме значений: ячейка 1024 блока, 3 октавы; пороги - по долям площади (Р19)
    int reliefCellLog2 = 14, reliefOctaves = 3;
    fx::q16 hillsFrom = -fx::ONE / 10, hillsTo = fx::ONE / 10;          // равнины -> холмы
    fx::q16 mountFrom = fx::ONE * 2 / 10, mountTo = fx::ONE * 4 / 10;   // холмы -> горы
    // климат
    int32_t climateBand = 20000 * VOXELS_PER_BLOCK;   // от самого холодного пояса до самого жаркого (как в TFC)
    fx::q16 tempMid = 8 * DEG, tempSwing = 22 * DEG;  // на уровне моря: от -14 до +30 °C по поясам
    fx::q16 tempNoise = 5 * DEG;                       // местные отклонения ±5 °C
    fx::q16 rainMid = 250 * DEG, rainSwing = 250 * DEG, rainNoise = 75 * DEG;   // осадки 0..500 мм в год
    int climateCellLog2 = 15, climateOctaves = 2;      // шум климата: ячейка 2048 блоков
    int32_t seaLevel = 32 * VOXELS_PER_BLOCK;          // от него отсчёт падения температуры с высотой
    fx::q16 lapsePerVoxel = 655;                       // 0.16 °C на блок = 0.01 °C на воксель
    fx::q16 snowBelow = -2 * DEG;                      // снег - где средняя температура ниже
    fx::q16 desertRain = 100 * DEG, desertTemp = 15 * DEG;   // пустыня: сухо и тепло
};

// Форма рельефа в столбце: высота поверхности (верхний твёрдый воксель) и веса трёх видов, сумма весов = 1
struct Relief {
    int32_t height;
    fx::q16 wPlains, wHills, wMount;
};
// minCellLog2 - для дальних уровней (Р20): октавы мельче 2^minCellLog2 вокселей не считаются
Relief reliefAt(const TerrainParams& p, int32_t x, int32_t z, int minCellLog2 = 0);

// Климат столбца на уровне моря: средняя годовая температура (°C) и осадки (мм в год), оба в Q16
struct Climate {
    fx::q16 temp, rain;
};
Climate climateAt(const TerrainParams& p, int32_t x, int32_t z);
fx::q16 temperatureAt(const TerrainParams& p, const Climate& c, int32_t height);   // с поправкой на высоту

// Покров: верхний воксель и слой под ним (3 блока)
struct Cover {
    Material top, under;
};
Cover coverAt(const TerrainParams& p, const Climate& c, int32_t height);

// Высоты столбцов куска С КАЙМОЙ в один воксель: [-1..32] по x и z, 34×34. Кайма нужна сетке: грань на краю
// куска видна, только если сосед за краем - воздух, а соседа считаем той же формулой, не трогая чужой кусок.
struct ColumnHeights {
    int32_t h[CHUNK + 2][CHUNK + 2];   // [z+1][x+1], высота поверхности в вокселях (верхний твёрдый воксель)
    Cover cover[CHUNK][CHUNK];         // [z][x], только без каймы: покров нужен лишь своим вокселям
    int32_t minH, maxH;
};

void columnHeights(const TerrainParams& p, int32_t chunkX, int32_t chunkZ, ColumnHeights& out);

// Заполнить кусок по высотам: камень, 3 блока подпокровного слоя, сверху воксель покрова
void fillChunk(const ColumnHeights& heights, Chunk& c);

// Контрольная сумма вокселей куска - для проверки воспроизводимости (Р2)
uint64_t checksum(const Chunk& c);

}  // namespace world
