#pragma once
// Шум в фиксированной точке (Р15, Р16, Р17, Р19).
// Координаты везде - в ВОКСЕЛЯХ (целые), ячейка - 2^cellLog2 вокселей (cellLog2 <= 16).
#include <cstdint>

#include "gen/fixed.h"

namespace gen {

// Градиентный шум Перлина. Ответ - Q16, примерно в [-1, 1]. Р15: равнины.
fx::q16 perlin2(int32_t x, int32_t z, int cellLog2, uint32_t seed);

// Симплекс-шум: треугольная сетка, 3 вершины вместо 4, без осевых швов. Ответ - Q16 в [-1, 1]. Р15: холмы, горы.
fx::q16 simplex2(int32_t x, int32_t z, int cellLog2, uint32_t seed);

// Шум значений: случайное число в узле, сглаженное смешивание. Самый дешёвый. Ответ - Q16 в [-1, 1]. Р15: маски.
fx::q16 value2(int32_t x, int32_t z, int cellLog2, uint32_t seed);

// Суммы октав (fBm): ячейка каждой следующей вдвое мельче (точно - сдвигом), размах - вдвое меньше.
// firstCellLog2 - размер ячейки первой октавы (11 -> 2048 вокселей = 128 блоков). Ответ - Q16 в [-1, 1].
fx::q16 fbm2(int32_t x, int32_t z, int firstCellLog2, int octaves, uint32_t seed);        // Перлин
fx::q16 fbmSimplex2(int32_t x, int32_t z, int firstCellLog2, int octaves, uint32_t seed);
fx::q16 fbmValue2(int32_t x, int32_t z, int firstCellLog2, int octaves, uint32_t seed);

// Хребты: октавы (1 - |симплекс|)², каждая следующая - с весом по предыдущей (мелкие гребни - на крупных,
// а в долинах гладко). Ответ - Q16 в [0, 1]: 1 - гребень, 0 - дно долины.
fx::q16 ridged2(int32_t x, int32_t z, int firstCellLog2, int octaves, uint32_t seed);

}  // namespace gen
