#pragma once
// Шум в фиксированной точке (Р15, Р16, Р17).
#include <cstdint>

#include "gen/fixed.h"

namespace gen {

// Градиентный шум Перлина в точке (x, z) - координаты в ВОКСЕЛЯХ (целые), ячейка 2^cellLog2 вокселей.
// Ответ - Q16, примерно в [-1, 1].
fx::q16 perlin2(int32_t x, int32_t z, int cellLog2, uint32_t seed);

// Сумма октав (fBm): ячейка каждой следующей вдвое мельче (точно - сдвигом), размах - вдвое меньше.
// firstCellLog2 - размер ячейки первой октавы (11 -> 2048 вокселей = 128 блоков). Ответ - Q16 в [-1, 1].
fx::q16 fbm2(int32_t x, int32_t z, int firstCellLog2, int octaves, uint32_t seed);

}  // namespace gen
