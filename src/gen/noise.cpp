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

// место внутри ячейки в Q16: младшие биты координаты, растянутые до [0, 1)
inline fx::q16 cellFrac(int32_t v, int cellLog2) {
    const int32_t mask = (1 << cellLog2) - 1;
    return cellLog2 <= 16 ? (v & mask) << (16 - cellLog2) : (v & mask) >> (cellLog2 - 16);
}

// значение узла для шума значений: 17 младших бит хеша -> [-1, 1)
inline fx::q16 nodeValue(int32_t cx, int32_t cz, uint32_t seed) {
    return static_cast<fx::q16>(hash2(cx, cz, seed) & 0x1FFFF) - fx::ONE;
}

// Направления для симплекса - СВОИ: 24 штуки через 15°, со сдвигом 7.5°. Рёбра треугольной сетки идут под
// -15°, 45° и 105°, перпендикуляры к ним - 15°, 75°, 135° (все кратны 15°). Таблица Перлина (через 22.5°)
// содержит точные перпендикуляры к диагональному ребру: если они выпали обеим вершинам ребра, шум равен нулю
// вдоль всего ребра, и хребты (Р19) вытягиваются в прямые линии. Здесь любое направление отстоит от
// перпендикуляра на 7.5° - нулевые линии не прилипают к рёбрам. (Так же устроена таблица OpenSimplex2.)
constexpr fx::q16 SGRAD[24][2] = {
    {64975, 8554},   {60547, 25080},   {51993, 39896},   {39896, 51993},
    {25080, 60547},  {8554, 64975},    {-8554, 64975},   {-25080, 60547},
    {-39896, 51993}, {-51993, 39896},  {-60547, 25080},  {-64975, 8554},
    {-64975, -8554}, {-60547, -25080}, {-51993, -39896}, {-39896, -51993},
    {-25080, -60547},{-8554, -64975},  {8554, -64975},   {25080, -60547},
    {39896, -51993}, {51993, -39896},  {60547, -25080},  {64975, -8554},
};

// Симплекс: перекос сетки F = (√3 - 1)/2 и обратный G = (3 - √3)/6, в Q16 - округлённые константы.
// Квадратная сетка, сжатая вдоль диагонали на F, становится сеткой равносторонних треугольников.
constexpr int64_t SKEW = 23988;     // 0.3660254 · 65536
constexpr int64_t UNSKEW = 13849;   // 0.2113249 · 65536
// Наибольшая сумма вкладов при единичных направлениях - 0.01008 (численный перебор точек треугольника, Р19):
// растягиваем до ±1 множителем 99.2 (Q16).
constexpr int64_t SIMPLEX_SCALE = 6501455;

// вклад вершины симплекса в Q32 (больше точности: (0.5 - d²)^4 доходит лишь до 1/16, в Q16 съело бы младшие биты)
inline int64_t simplexCorner(int64_t i, int64_t j, uint32_t seed, int64_t dx, int64_t dz) {
    const int64_t t = (fx::ONE >> 1) - ((dx * dx + dz * dz) >> 16);   // Q16: 0.5 - d²
    if (t <= 0) return 0;
    const int64_t t2 = t * t;                       // Q32
    const int64_t t4 = (t2 * t2) >> 32;             // Q32, не больше 2^28
    // номер из 24 без деления: старшая часть произведения хеша на 24 (хеш · 24 / 2^32)
    const uint32_t h = hash2(static_cast<int32_t>(i), static_cast<int32_t>(j), seed);
    const fx::q16* g = SGRAD[(uint64_t(h) * 24) >> 32];
    const int64_t dot = (g[0] * dx + g[1] * dz) >> 16;   // Q16
    return (t4 * dot) >> 16;                        // Q32
}

inline fx::q16 clampOne(int64_t v) {
    return static_cast<fx::q16>(v > fx::ONE ? fx::ONE : (v < -fx::ONE ? -fx::ONE : v));
}

// Общая сумма октав для любого базового шума: размахи 1, 1/2, 1/4, ... - сдвигом; их сумма 2 - 2^(1-n),
// нормируем делением на неё.
template <fx::q16 (*Noise)(int32_t, int32_t, int, uint32_t)>
fx::q16 fbm(int32_t x, int32_t z, int firstCellLog2, int octaves, uint32_t seed, int minCellLog2) {
    int64_t sum = 0, norm = 0;
    for (int o = 0; o < octaves && firstCellLog2 - o >= 1; ++o) {
        norm += static_cast<int64_t>(fx::ONE) >> o;
        if (firstCellLog2 - o < minCellLog2) continue;   // слишком мелкая для этого уровня - выпадает
        const fx::q16 n = Noise(x, z, firstCellLog2 - o, octaveSeed(seed, o));
        sum += static_cast<int64_t>(n) >> o;
    }
    return static_cast<fx::q16>((sum << fx::SHIFT) / norm);
}

}  // namespace

fx::q16 perlin2(int32_t x, int32_t z, int cellLog2, uint32_t seed) {
    // Номер ячейки и место в ней - без единого дробного умножения: ячейка - степень двойки.
    // Сдвиг вправо у отрицательных - арифметический, поэтому -1 >> k = -1: ячейка левее нуля, верно.
    const int32_t cx = x >> cellLog2, cz = z >> cellLog2;
    const fx::q16 fx0 = cellFrac(x, cellLog2), fz0 = cellFrac(z, cellLog2);
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

fx::q16 simplex2(int32_t x, int32_t z, int cellLog2, uint32_t seed) {
    // Координата в ячейках, Q16 в 64 битах: x · 2^(16 - k) - сдвиг, без деления
    // (ячейка крупнее 2^16 вокселей - сдвигом вправо: доля ячейки грубее вокселя, для медленных полей не важно)
    const int64_t X = cellLog2 <= 16 ? static_cast<int64_t>(x) << (16 - cellLog2) : static_cast<int64_t>(x) >> (cellLog2 - 16);
    const int64_t Z = cellLog2 <= 16 ? static_cast<int64_t>(z) << (16 - cellLog2) : static_cast<int64_t>(z) >> (cellLog2 - 16);
    // перекос: в каком ромбе квадратной сетки точка; сдвиг вправо - округление вниз и для отрицательных
    const int64_t s = ((X + Z) * SKEW) >> 16;
    const int64_t i = (X + s) >> 16, j = (Z + s) >> 16;
    // обратно: смещение точки от первой вершины треугольника
    const int64_t t = (i + j) * UNSKEW;
    const int64_t x0 = X - ((i << 16) - t), z0 = Z - ((j << 16) - t);
    // какой из двух треугольников ромба: ниже диагонали или выше
    const int64_t i1 = x0 > z0 ? 1 : 0, j1 = 1 - i1;
    const int64_t x1 = x0 - (i1 << 16) + UNSKEW, z1 = z0 - (j1 << 16) + UNSKEW;
    const int64_t x2 = x0 - fx::ONE + 2 * UNSKEW, z2 = z0 - fx::ONE + 2 * UNSKEW;

    const int64_t sum = simplexCorner(i, j, seed, x0, z0) + simplexCorner(i + i1, j + j1, seed, x1, z1) +
                        simplexCorner(i + 1, j + 1, seed, x2, z2);   // Q32
    return clampOne((sum * SIMPLEX_SCALE) >> 32);
}

namespace {
// 3D: перекос 1/3, обратный 1/6; направления - 12 рёбер куба (длина √2). Вклад вершины (0.5 - d²)^4: с 0.5
// (а не 0.6, как у классики) вклад гаснет до соседних тетраэдров - без разрывов на гранях.
constexpr int64_t SKEW3 = 21845, UNSKEW3 = 10923;
// наибольшая сумма вкладов при направлениях вдоль смещения - 0.01314 (численный перебор, Р26): ×76.1
constexpr int64_t SIMPLEX3_SCALE = 4989093;
constexpr int8_t GRAD3[12][3] = {{1, 1, 0}, {-1, 1, 0}, {1, -1, 0}, {-1, -1, 0}, {1, 0, 1}, {-1, 0, 1},
                                 {1, 0, -1}, {-1, 0, -1}, {0, 1, 1}, {0, -1, 1}, {0, 1, -1}, {0, -1, -1}};

inline int64_t simplex3Corner(int64_t i, int64_t j, int64_t k, uint32_t seed, int64_t dx, int64_t dy, int64_t dz) {
    const int64_t t = (fx::ONE >> 1) - ((dx * dx + dy * dy + dz * dz) >> 16);
    if (t <= 0) return 0;
    const int64_t t2 = t * t;
    const int64_t t4 = (t2 * t2) >> 32;   // Q32
    const uint32_t h = hash3(static_cast<int32_t>(i), static_cast<int32_t>(j), static_cast<int32_t>(k), seed);
    const int8_t* g = GRAD3[(uint64_t(h) * 12) >> 32];
    const int64_t dot = g[0] * dx + g[1] * dy + g[2] * dz;   // Q16
    return (t4 * dot) >> 16;               // Q32
}
}  // namespace

fx::q16 simplex3(int32_t x, int32_t y, int32_t z, int cellLog2, uint32_t seed) {
    const auto cell = [&](int32_t v) {
        return cellLog2 <= 16 ? static_cast<int64_t>(v) << (16 - cellLog2) : static_cast<int64_t>(v) >> (cellLog2 - 16);
    };
    const int64_t X = cell(x), Y = cell(y), Z = cell(z);
    const int64_t s = ((X + Y + Z) * SKEW3) >> 16;
    const int64_t i = (X + s) >> 16, j = (Y + s) >> 16, k = (Z + s) >> 16;
    const int64_t t = (i + j + k) * UNSKEW3;
    const int64_t x0 = X - ((i << 16) - t), y0 = Y - ((j << 16) - t), z0 = Z - ((k << 16) - t);
    // какой из шести тетраэдров куба: по порядку смещений
    int i1, j1, k1, i2, j2, k2;
    if (x0 >= y0) {
        if (y0 >= z0)      { i1 = 1; j1 = 0; k1 = 0; i2 = 1; j2 = 1; k2 = 0; }
        else if (x0 >= z0) { i1 = 1; j1 = 0; k1 = 0; i2 = 1; j2 = 0; k2 = 1; }
        else               { i1 = 0; j1 = 0; k1 = 1; i2 = 1; j2 = 0; k2 = 1; }
    } else {
        if (y0 < z0)       { i1 = 0; j1 = 0; k1 = 1; i2 = 0; j2 = 1; k2 = 1; }
        else if (x0 < z0)  { i1 = 0; j1 = 1; k1 = 0; i2 = 0; j2 = 1; k2 = 1; }
        else               { i1 = 0; j1 = 1; k1 = 0; i2 = 1; j2 = 1; k2 = 0; }
    }
    const int64_t sum =
        simplex3Corner(i, j, k, seed, x0, y0, z0) +
        simplex3Corner(i + i1, j + j1, k + k1, seed, x0 - (int64_t(i1) << 16) + UNSKEW3, y0 - (int64_t(j1) << 16) + UNSKEW3,
                       z0 - (int64_t(k1) << 16) + UNSKEW3) +
        simplex3Corner(i + i2, j + j2, k + k2, seed, x0 - (int64_t(i2) << 16) + 2 * UNSKEW3,
                       y0 - (int64_t(j2) << 16) + 2 * UNSKEW3, z0 - (int64_t(k2) << 16) + 2 * UNSKEW3) +
        simplex3Corner(i + 1, j + 1, k + 1, seed, x0 - fx::ONE + 3 * UNSKEW3, y0 - fx::ONE + 3 * UNSKEW3,
                       z0 - fx::ONE + 3 * UNSKEW3);
    return clampOne((sum * SIMPLEX3_SCALE) >> 32);
}

fx::q16 value2(int32_t x, int32_t z, int cellLog2, uint32_t seed) {
    const int32_t cx = x >> cellLog2, cz = z >> cellLog2;
    const fx::q16 u = fx::fade(cellFrac(x, cellLog2)), w = fx::fade(cellFrac(z, cellLog2));
    const fx::q16 top = fx::lerp(nodeValue(cx, cz, seed), nodeValue(cx + 1, cz, seed), u);
    const fx::q16 bot = fx::lerp(nodeValue(cx, cz + 1, seed), nodeValue(cx + 1, cz + 1, seed), u);
    return fx::lerp(top, bot, w);
}

fx::q16 fbm2(int32_t x, int32_t z, int firstCellLog2, int octaves, uint32_t seed, int minCellLog2) {
    return fbm<perlin2>(x, z, firstCellLog2, octaves, seed, minCellLog2);
}
fx::q16 fbmSimplex2(int32_t x, int32_t z, int firstCellLog2, int octaves, uint32_t seed, int minCellLog2) {
    return fbm<simplex2>(x, z, firstCellLog2, octaves, seed, minCellLog2);
}
fx::q16 fbmValue2(int32_t x, int32_t z, int firstCellLog2, int octaves, uint32_t seed, int minCellLog2) {
    return fbm<value2>(x, z, firstCellLog2, octaves, seed, minCellLog2);
}

fx::q16 ridged2(int32_t x, int32_t z, int firstCellLog2, int octaves, uint32_t seed, int minCellLog2, bool squared,
                fx::q16 gain) {
    // Гребень там, где шум проходит через ноль: 1 - |n| даёт острый излом, квадрат - сужает гребень.
    // Вес следующей октавы = 2 · (вклад предыдущей), не больше 1: на склонах и гребнях - мелкие гребешки,
    // в долинах (вклад мал) мелочь гаснет - дно ровное, как у настоящих гор.
    // размах октавы amp: 1, gain, gain², ... (при gain = 0.5 - ровно прежние сдвиги: 2^16 делится нацело)
    int64_t sum = 0, norm = 0;
    int64_t weight = fx::ONE;
    int64_t amp = fx::ONE;
    for (int o = 0; o < octaves && firstCellLog2 - o >= 1; ++o, amp = (amp * gain) >> fx::SHIFT) {
        norm += amp;
        if (firstCellLog2 - o < minCellLog2) break;   // дальше только мельче
        const fx::q16 n = simplex2(x, z, firstCellLog2 - o, octaveSeed(seed, o));
        int64_t r = fx::ONE - (n < 0 ? -n : n);   // [0, 1]
        if (squared) r = (r * r) >> 16;
        r = (r * weight) >> 16;
        weight = r * 2 > fx::ONE ? fx::ONE : r * 2;
        sum += (r * amp) >> fx::SHIFT;
    }
    return static_cast<fx::q16>((sum << fx::SHIFT) / norm);
}

}  // namespace gen
