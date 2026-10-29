#include "gen/noise.h"

#include "gen/hash.h"

namespace gen {
namespace {

// 16 направлений на окружности, cos/sin, умноженные на 65536 и округлённые. Таблица - константа:
// никакой тригонометрии при расчёте, значит и никаких расхождений между машинами.
constexpr fx::q16 GRAD[16][2] = {
    {65536, 0},      {60547, 25080},  {46341, 46341},  {25080, 60547},
    {0, 65536},      {-25080, 60547}, {-46341, 46341}, {-60547, 25080},
    {-65536, 0},     {-60547, -25080},{-46341, -46341},{-25080, -60547},
    {0, -65536},     {25080, -60547}, {46341, -46341}, {60547, -25080},
};

// вклад узла: скалярное произведение его случайного направления на смещение точки от узла
inline fx::q16 cornerDot(int32_t cx, int32_t cz, uint32_t seed, fx::q16 dx, fx::q16 dz) {
    const fx::q16* g = GRAD[hash2(cx, cz, seed) & 15];
    return fx::mul(g[0], dx) + fx::mul(g[1], dz);
}

}  // namespace

fx::q16 perlin2(int32_t x, int32_t z, int cellLog2, uint32_t seed) {
    // Номер ячейки и место в ней - без единого дробного умножения: ячейка - степень двойки.
    // Сдвиг вправо у отрицательных - арифметический, поэтому -1 >> k = -1: ячейка левее нуля, верно.
    const int32_t cx = x >> cellLog2, cz = z >> cellLog2;
    const int32_t mask = (1 << cellLog2) - 1;
    // место внутри ячейки в Q16: младшие биты координаты, растянутые до [0, 1)
    const fx::q16 fx0 = cellLog2 <= 16 ? (x & mask) << (16 - cellLog2) : (x & mask) >> (cellLog2 - 16);
    const fx::q16 fz0 = cellLog2 <= 16 ? (z & mask) << (16 - cellLog2) : (z & mask) >> (cellLog2 - 16);
    const fx::q16 fx1 = fx0 - fx::ONE, fz1 = fz0 - fx::ONE;

    const fx::q16 n00 = cornerDot(cx, cz, seed, fx0, fz0);
    const fx::q16 n10 = cornerDot(cx + 1, cz, seed, fx1, fz0);
    const fx::q16 n01 = cornerDot(cx, cz + 1, seed, fx0, fz1);
    const fx::q16 n11 = cornerDot(cx + 1, cz + 1, seed, fx1, fz1);

    const fx::q16 u = fx::fade(fx0), w = fx::fade(fz0);
    const fx::q16 top = fx::lerp(n00, n10, u);
    const fx::q16 bot = fx::lerp(n01, n11, u);
    // У 2D-Перлина с единичными градиентами размах ±0.707: растягиваем до ±1 (×1.414 ≈ 92682/65536)
    return fx::mul(fx::lerp(top, bot, w), 92682);
}

fx::q16 fbm2(int32_t x, int32_t z, int firstCellLog2, int octaves, uint32_t seed) {
    // Размахи 1, 1/2, 1/4, ... - сдвигом; их сумма 2 - 2^(1-n), нормируем делением на неё.
    int64_t sum = 0, norm = 0;
    for (int o = 0; o < octaves && firstCellLog2 - o >= 1; ++o) {
        const fx::q16 n = perlin2(x, z, firstCellLog2 - o, octaveSeed(seed, o));
        sum += static_cast<int64_t>(n) >> o;
        norm += static_cast<int64_t>(fx::ONE) >> o;
    }
    return static_cast<fx::q16>((sum << fx::SHIFT) / norm);
}

}  // namespace gen
