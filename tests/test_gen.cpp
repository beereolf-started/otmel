// Тесты генерации (Р6): то, что проверяется машиной. Главное - воспроизводимость от зерна (Р2, Р16).
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "gen/fixed.h"
#include "gen/hash.h"
#include "gen/noise.h"
#include "world/bits.h"
#include "world/caves.h"
#include "world/rivers.h"
#include "world/lod.h"
#include "world/mesher.h"
#include "world/terrain.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <tuple>
#include <vector>

namespace world {
void buildMeshReference(const Chunk& c, const ColumnHeights& hm, std::vector<uint32_t>& out, MeshStats& stats);
}

// Набор прямоугольников сетки как множество: порядок выдачи у разных способов разный, а набор - обязан совпасть
static std::vector<std::array<uint32_t, 4>> quadSet(const std::vector<uint32_t>& v) {
    std::vector<std::array<uint32_t, 4>> q;
    for (size_t i = 0; i + 3 < v.size(); i += 4) q.push_back({v[i], v[i + 1], v[i + 2], v[i + 3]});
    std::sort(q.begin(), q.end());
    return q;
}

static int g_fail = 0;
#define CHECK(cond, what)                                              \
    do {                                                               \
        if (cond) std::printf("годен     %s\n", what);                 \
        else { std::printf("НЕ ГОДЕН  %s\n", what); ++g_fail; }      \
    } while (0)

// Р36: у начала мира - океан (континенты в океане); кусок на суше - от точки старта вглубь материка (на запад)
static void landChunk(const world::TerrainParams& p, int& cx, int& cz) {
    cx = -12450 / 2;
    cz = 23560 / 2;
    world::ColumnHeights hm;
    for (int k = 0; k < 400; ++k, cx -= 25) {
        world::columnHeights(p, cx, cz, hm);
        if (hm.minH > p.seaLevel + 16 * world::VOXELS_PER_BLOCK) return;
    }
}

static uint64_t chunkSum(const world::TerrainParams& p, int cx, int cy, int cz) {
    world::ColumnHeights hm;
    world::columnHeights(p, cx, cz, hm);
    world::Chunk c;
    c.pos = {cx, cy, cz};
    world::fillChunk(hm, c);
    return world::checksum(c);
}

int main() {
    // фиксированная точка
    CHECK(fx::mul(fx::ONE / 2, fx::ONE / 2) == fx::ONE / 4, "0.5 * 0.5 = 0.25");
    CHECK(fx::fade(0) == 0 && fx::fade(fx::ONE) == fx::ONE, "сглаживание: 0 -> 0, 1 -> 1");
    CHECK(fx::fade(fx::ONE / 2) == fx::ONE / 2, "сглаживание симметрично: 0.5 -> 0.5");

    // хеш: соседние координаты - разные числа
    CHECK(gen::hash2(5, 7, 1) != gen::hash2(6, 7, 1), "хеш: соседи различаются");
    CHECK(gen::hash2(5, 7, 1) == gen::hash2(5, 7, 1), "хеш: одно и то же - одинаково");

    // шум: на узлах ячейки - ноль (так устроен градиентный шум), в пределах [-1, 1]
    CHECK(gen::perlin2(0, 0, 6, 1) == 0 && gen::perlin2(64, 128, 6, 1) == 0, "Перлин: в узлах сетки ноль");
    bool inRange = true;
    for (int i = -500; i < 500; i += 7)
        for (int j = -500; j < 500; j += 11) {
            const fx::q16 n = gen::fbm2(i, j, 11, 6, 42);
            if (n < -fx::ONE || n > fx::ONE) inRange = false;
        }
    CHECK(inRange, "сумма октав в [-1, 1]");

    // ─── Р19: новые шумы ─────────────────────────────────────────────────────────────
    {
        // симплекс: в [-1, 1], множитель растяжения верен (наибольшее близко к 1, обрезка почти не срабатывает),
        // непрерывен (соседние воксели - почти одно значение) и в среднем около нуля
        fx::q16 lo = 0, hi = 0;
        int clipped = 0, samples = 0;
        int64_t sum = 0;
        fx::q16 maxStep = 0;
        for (int i = -3000; i < 3000; i += 3)
            for (int j = -3000; j < 3000; j += 7) {
                const fx::q16 n = gen::simplex2(i, j, 8, 7);
                lo = std::min(lo, n); hi = std::max(hi, n);
                if (n == fx::ONE || n == -fx::ONE) ++clipped;
                sum += n; ++samples;
                maxStep = std::max(maxStep, std::abs(gen::simplex2(i + 1, j, 8, 7) - n));
            }
        std::printf("симплекс: от %.3f до %.3f, среднее %.4f, наибольший шаг между соседними вокселями %.4f\n",
                    lo / 65536.0, hi / 65536.0, double(sum) / samples / 65536.0, maxStep / 65536.0);
        CHECK(lo >= -fx::ONE && hi <= fx::ONE && hi > fx::ONE * 85 / 100 && lo < -fx::ONE * 85 / 100,
              "симплекс: размах почти весь [-1, 1] и не шире");
        CHECK(clipped * 1000 < samples, "симплекс: обрезка по краю - реже 0.1% точек");
        CHECK(maxStep < fx::ONE / 20, "симплекс непрерывен: шаг на воксель меньше 0.05 при ячейке 256");

        // шум значений: в [-1, 1]
        bool valueOk = true;
        for (int i = -40; i < 40; ++i) {
            const fx::q16 n = gen::value2(i * 37, i * 53, 6, 3);
            if (n < -fx::ONE || n > fx::ONE) valueOk = false;
        }
        CHECK(valueOk, "шум значений в [-1, 1]");

        // хребты: в [0, 1]
        fx::q16 rlo = fx::ONE, rhi = 0;
        for (int i = -4000; i < 4000; i += 13)
            for (int j = -4000; j < 4000; j += 17) {
                const fx::q16 r = gen::ridged2(i, j, 13, 7, 9);
                rlo = std::min(rlo, r); rhi = std::max(rhi, r);
            }
        std::printf("хребты: от %.3f до %.3f\n", rlo / 65536.0, rhi / 65536.0);
        CHECK(rlo >= 0 && rhi <= fx::ONE, "хребты в [0, 1]");

        // плавная ступень: концы, середина, монотонность
        bool mono = true;
        for (fx::q16 v = -fx::ONE, prev = -1; v <= fx::ONE; v += 97) {
            const fx::q16 s = fx::smoothstep(-fx::ONE / 2, fx::ONE / 2, v);
            if (s < prev) mono = false;
            prev = s;
        }
        CHECK(fx::smoothstep(0, fx::ONE, 0) == 0 && fx::smoothstep(0, fx::ONE, fx::ONE) == fx::ONE &&
                  fx::smoothstep(0, fx::ONE, fx::ONE / 2) == fx::ONE / 2 && mono,
              "плавная ступень: 0 -> 0, 1 -> 1, середина -> 0.5, не убывает");
    }

    // ─── Р19: смешение рельефа и климат ───────────────────────────────────────────────
    {
        // веса: каждый в [0, 1], сумма ровно 1; на длинном проходе через все зоны - ни одного скачка
        // ни веса, ни высоты (ступенька на границе зон дала бы скачок в десятки блоков)
        world::TerrainParams tp;
        bool sumOk = true, rangeOk = true;
        fx::q16 maxW = 0;
        int32_t maxStep = 0, maxStepPure = 0;
        int zonesSeen[3] = {};
        world::Relief prev = world::reliefAt(tp, -400000, 12345);
        for (int32_t x = -400000 + 1; x < 400000; ++x) {
            const world::Relief r = world::reliefAt(tp, x, 12345);
            if (r.wPlains + r.wHills + r.wMount != fx::ONE) sumOk = false;
            for (fx::q16 w : {r.wPlains, r.wHills, r.wMount}) if (w < 0 || w > fx::ONE) rangeOk = false;
            maxW = std::max({maxW, std::abs(r.wPlains - prev.wPlains), std::abs(r.wHills - prev.wHills),
                             std::abs(r.wMount - prev.wMount)});
            // Р32: на кромке реки берег до блока над водой - это берег, а не стык зон: пары вода/суша не считаем.
            // Р41: и пары у реки (берег): у горной реки берег врезан - ступень до 3 блоков у кромки русла задумана
            const bool bank = r.shore || prev.shore || (r.water > r.height) != (prev.water > prev.height);
            const int32_t step = bank ? 0 : std::abs(r.height - prev.height);
            maxStep = std::max(maxStep, step);
            if (r.wMount == fx::ONE && prev.wMount == fx::ONE) maxStepPure = std::max(maxStepPure, step);
            if (r.wPlains == fx::ONE) zonesSeen[0] = 1;
            if (r.wHills > fx::ONE * 9 / 10) zonesSeen[1] = 1;
            if (r.wMount == fx::ONE) zonesSeen[2] = 1;
            prev = r;
        }
        std::printf("проход 50000 блоков: наибольшее изменение веса на воксель %.5f; наибольший перепад высоты "
                    "между соседними вокселями %d (в чистых горах %d)\n", maxW / 65536.0, maxStep, maxStepPure);
        CHECK(sumOk, "веса видов рельефа в сумме ровно 1");
        CHECK(rangeOk, "каждый вес в [0, 1]");
        CHECK(zonesSeen[0] && zonesSeen[1] && zonesSeen[2], "на проходе встречаются равнины, холмы и горы");
        // переход от 0 до 1 занимает не меньше 1/maxW вокселей: требуем не уже 32 блоков (512 вокселей)
        std::printf("самый узкий переход между зонами - около %.0f блоков\n", 65536.0 / std::max(maxW, 1) / 16.0);
        CHECK(maxW < fx::ONE / 512, "переход между зонами не уже 32 блоков");
        CHECK(maxStep <= std::max(maxStepPure, 16), "на стыке зон нет ступеньки: перепад не больше, чем в самих горах");

        // климат: выше - холоднее; пояса по z - от холодного к жаркому; осадки в 0..500
        const world::Climate c = world::climateAt(tp, 0, 0);
        CHECK(world::temperatureAt(tp, c, tp.seaLevel + 160) < world::temperatureAt(tp, c, tp.seaLevel),
              "на 10 блоков выше уровня моря - холоднее");
        CHECK(world::temperatureAt(tp, c, tp.seaLevel - 160) == world::temperatureAt(tp, c, tp.seaLevel),
              "ниже уровня моря - как на уровне моря");
        const world::Climate hot = world::climateAt(tp, 0, tp.climateBand / 2), cold = world::climateAt(tp, 0, -tp.climateBand / 2);
        std::printf("климат: в начале %.1f °C, %.0f мм; на жарком поясе %.1f °C, на холодном %.1f °C\n",
                    c.temp / 65536.0, c.rain / 65536.0, hot.temp / 65536.0, cold.temp / 65536.0);
        CHECK(hot.temp > 20 * world::DEG && cold.temp < -5 * world::DEG, "пояса температуры: к +z жарко, к -z холодно");
        bool rainOk = true;
        for (int32_t x = -800000; x < 800000; x += 997)
            if (const auto cl = world::climateAt(tp, x, x / 3); cl.rain < 0 || cl.rain > 500 * world::DEG) rainOk = false;
        CHECK(rainOk, "осадки в пределах 0..500 мм");
    }

    // воспроизводимость: один и тот же кусок - одна и та же сумма, в том числе на отрицательных координатах
    world::TerrainParams p;
    // кусок, через который проходит поверхность (а не сплошной воздух или камень - те у любых зёрен одинаковы),
    // на суше (Р36: у начала мира - океан, дно у разных зёрен может совпасть)
    int lx, lz;
    landChunk(p, lx, lz);
    int cy0;
    {
        world::ColumnHeights hm;
        world::columnHeights(p, lx, lz, hm);
        cy0 = hm.maxH >> world::CHUNK_LOG2;
    }
    const uint64_t a = chunkSum(p, lx, cy0, lz), b = chunkSum(p, lx, cy0, lz);
    CHECK(a == b, "кусок из одного зерна дважды - одинаков");
    world::TerrainParams q = p;
    q.seed = p.seed + 1;
    CHECK(chunkSum(q, lx, cy0, lz) != a, "другое зерно - другой кусок");
    // Эта сумма должна совпадать в отладочной и быстрой сборке и на любой машине (Р2, Р16)
    std::printf("сумма куска (%d,%d,%d) при зерне 1337: %016llx\n", lx, cy0, lz, static_cast<unsigned long long>(a));
    {
        // горный кусок у самой высокой вершины округи (ищем сами - тест не зависит от того, где сейчас горы):
        // хребты, симплекс, пояс цепи, смешение весов, климат и снег - тоже до бита
        int bx = 0, bz = 0, best = INT32_MIN;
        for (int z = -16; z < 16; ++z)
            for (int x = -16; x < 16; ++x) {
                const int32_t h = world::reliefAt(p, x * 16 * 1024, z * 16 * 1024).height;   // через 1024 блока
                if (h > best) { best = h; bx = x * 512; bz = z * 512; }
            }
        world::ColumnHeights hm;
        world::columnHeights(p, bx, bz, hm);
        const int cy = (hm.maxH >> world::CHUNK_LOG2) - 1;
        bool snow = false;
        for (auto& row : hm.cover) for (auto& cv : row) if (cv.top == world::SNOW) snow = true;
        CHECK(snow && hm.maxH > 150 * world::VOXELS_PER_BLOCK, "у вершины цепи - горы выше 150 блоков и снег");
        std::printf("сумма горного куска (%d,%d,%d) при зерне 1337: %016llx (вершина %.0f блоков)\n", bx, cy, bz,
                    static_cast<unsigned long long>(chunkSum(p, bx, cy, bz)), hm.maxH / 16.0);
    }

    // поворот битовой таблицы 32×32 - против побитового эталона на случайных таблицах
    {
        uint32_t seed = 12345;
        bool ok = true;
        for (int n = 0; n < 200 && ok; ++n) {
            uint32_t m[32], ref[32] = {};
            for (int r = 0; r < 32; ++r) { seed = seed * 1664525u + 1013904223u; m[r] = seed; }
            for (int r = 0; r < 32; ++r)
                for (int col = 0; col < 32; ++col)
                    if (m[r] >> col & 1) ref[col] |= 1u << r;
            world::transpose32(m);
            for (int r = 0; r < 32; ++r) if (m[r] != ref[r]) ok = false;
        }
        CHECK(ok, "поворот таблицы 32x32 = побитовый эталон (200 случайных таблиц)");
    }

    // новая сетка (Р18) - тот же набор прямоугольников, что у прежней; и по формуле, и из вокселей
    {
        world::TerrainParams tp;
        world::ColumnHeights hm;
        world::Chunk c;
        std::vector<uint32_t> fast, ref;
        world::MeshStats ms;
        bool same = true, sameEdited = true, sameNoFill = true;
        int chunks = 0;
        int64_t waterQuads = 0;
        double fastMs = 0, refMs = 0;
        using clk = std::chrono::steady_clock;
        // Р34: кроме кусков у старта - квадрат 8×8 кусков у берега первой найденной воды (вода - своим проходом)
        int32_t wx = 0, wz = 0;
        for (int k = 0; k < 4000 && !wx; ++k) {
            world::columnHeights(tp, k * 3, 0, hm);
            for (int x = 1; x <= world::CHUNK && !wx; ++x)
                if (hm.water[1][x] > hm.h[1][x] && hm.water[1][x] - hm.h[1][x] < 4 * world::VOXELS_PER_BLOCK) wx = k * 3;
        }
        std::vector<std::pair<int32_t, int32_t>> cols;
        for (int cz = -5; cz < 5; ++cz)
            for (int cx = -5; cx < 5; ++cx) cols.push_back({cx, cz});
        for (int cz = -4; cz < 4; ++cz)
            for (int cx = -4; cx < 4; ++cx) cols.push_back({wx + cx, wz + cz});
        for (const auto& [cx, cz] : cols) {
                world::columnHeights(tp, cx, cz, hm);
                for (int cy = (hm.minH - 1) >> 5; cy <= (hm.maxH >> 5); ++cy) {
                    c.pos = {cx, cy, cz};
                    world::fillChunk(hm, c);
                    c.edited = false;
                    auto t0 = clk::now();
                    world::buildMesh(c, hm, fast, ms);
                    fastMs += std::chrono::duration<double, std::milli>(clk::now() - t0).count();
                    auto t1 = clk::now();
                    world::buildMeshReference(c, hm, ref, ms);
                    refMs += std::chrono::duration<double, std::milli>(clk::now() - t1).count();
                    if (quadSet(fast) != quadSet(ref)) same = false;
                    world::buildMesh(c.pos, hm, fast, ms);   // без вокселей вовсе (Р22)
                    waterQuads += ms.waterQuads;
                    if (quadSet(fast) != quadSet(ref)) sameNoFill = false;
                    c.edited = true;      // тот же кусок, но маски - из вокселей
                    world::buildMesh(c, hm, fast, ms);
                    if (quadSet(fast) != quadSet(ref)) sameEdited = false;
                    ++chunks;
                }
            }
        CHECK(same, "сетка по формуле = прежняя сетка (те же прямоугольники)");
        CHECK(sameEdited, "сетка из вокселей = прежняя сетка");
        CHECK(sameNoFill, "сетка без заполнения вокселей (материал формулой) = прежняя сетка");
        std::printf("сетка: у воды (кусок x %d) - %lld прямоугольников воды\n", wx, (long long)waterQuads);
        CHECK(waterQuads > 0, "в проверке сетки есть вода (дно и гладь - отдельно, Р34)");
        std::printf("сетка: %d кусков; прежняя %.3f мс на кусок, новая %.3f мс на кусок (в %.1f раза быстрее)\n",
                    chunks, refMs / chunks, fastMs / chunks, refMs / std::max(fastMs, 1e-9));
    }

    // замер цены столбца высот (один поток): равнины у начала и горы у вершины
    {
        world::TerrainParams tp;
        world::ColumnHeights hm;
        using clk = std::chrono::steady_clock;
        for (const auto& [name, bx, bz] : {std::tuple{"равнины", 0, 0}, std::tuple{"горы", 1948, 414}}) {
            // по частям: высоты, заполнение кусков, сетки - на 256 столбцах
            double tH = 0, tF = 0, tM = 0;
            int n = 0, chunks = 0;
            world::Chunk ch;
            std::vector<uint32_t> verts;
            world::MeshStats ms;
            for (int dz = -8; dz < 8; ++dz)
                for (int dx = -8; dx < 8; ++dx, ++n) {
                    auto t0 = clk::now();
                    world::columnHeights(tp, bx + dx, bz + dz, hm);
                    tH += std::chrono::duration<double, std::milli>(clk::now() - t0).count();
                    for (int cy = (hm.minH - 1) >> 5; cy <= (hm.maxH >> 5); ++cy, ++chunks) {
                        ch.pos = {bx + dx, cy, bz + dz};
                        ch.edited = false;
                        auto t2 = clk::now();
                        world::buildMesh(ch.pos, hm, verts, ms);   // без заполнения (Р22)
                        tM += std::chrono::duration<double, std::milli>(clk::now() - t2).count();
                    }
                }
            std::printf("столбец (%s), один поток: высоты %.3f мс + заполнение %.3f мс + сетки %.3f мс = %.3f мс (кусков в столбце %.2f)\n",
                        name, tH / n, tF / n, tM / n, (tH + tF + tM) / n, double(chunks) / n);
        }
    }

    // ─── Р26: пещеры ──────────────────────────────────────────────────────────────────────
    {
        // 3D-симплекс: в [-1, 1], растяжение верное, непрерывен
        fx::q16 lo = 0, hi = 0, maxStep = 0;
        for (int i = -600; i < 600; i += 5)
            for (int j = -600; j < 600; j += 7)
                for (int k = -300; k < 300; k += 23) {
                    const fx::q16 n = gen::simplex3(i, k, j, 8, 11);
                    lo = std::min(lo, n); hi = std::max(hi, n);
                    maxStep = std::max(maxStep, std::abs(gen::simplex3(i + 1, k, j, 8, 11) - n));
                }
        std::printf("3D-симплекс: от %.3f до %.3f, наибольший шаг на воксель %.4f\n", lo / 65536.0, hi / 65536.0, maxStep / 65536.0);
        CHECK(lo >= -fx::ONE && hi <= fx::ONE && hi > fx::ONE * 6 / 10 && lo < -fx::ONE * 6 / 10, "3D-симплекс: размах большой и не шире [-1, 1]");
        CHECK(maxStep < fx::ONE / 20, "3D-симплекс непрерывен");

        // пещеры: ищем куски с воздухом под землёй у начала мира; сетка такого куска - точна:
        // площадь её прямоугольников = число видимых граней, посчитанных в лоб по вокселям (с каймой)
        world::TerrainParams tp;
        static uint64_t air[world::CHUNK + 2][world::CHUNK + 2], air2[world::CHUNK + 2][world::CHUNK + 2];
        world::ColumnHeights hm;
        int checked = 0, caveChunks = 0, bestRun = 0;
        int32_t camAt[3] = {};   // для снимка: воксель в самом высоком проёме
        bool exact = true, repeat = true;
        int64_t airVox = 0, allVox = 0, shallowAir = 0, shallowAll = 0;
        std::vector<uint32_t> mesh;
        world::MeshStats ms;
        int clx, clz;
        landChunk(tp, clx, clz);   // Р36: на суше - у начала мира океан
        for (int cz = -6; cz < 6 && checked < 40; ++cz)
            for (int cx = -6; cx < 6 && checked < 40; ++cx) {
                world::columnHeights(tp, clx + cx * 7, clz + cz * 7, hm);
                const int cyLo = (hm.minH - 1) >> world::CHUNK_LOG2;
                for (int cy = cyLo - 20; cy <= cyLo; ++cy) {
                    const world::ChunkPos pos{clx + cx * 7, cy, clz + cz * 7};
                    if (!world::caveAir(tp, pos, hm, air)) {
                        // без пещер: все воксели под поверхностью - камень (для честной доли пустот)
                        for (int z = 1; z <= world::CHUNK; ++z)
                            for (int x = 1; x <= world::CHUNK; ++x) {
                                const int32_t top = hm.h[z][x], y0c = cy * world::CHUNK;
                                const int64_t n = std::clamp<int64_t>(int64_t(top) - y0c + 1, 0, world::CHUNK);
                                allVox += n;
                                const int64_t sh = std::clamp<int64_t>(int64_t(top) - std::max<int64_t>(y0c, top - tp.caves.tunnelCover + 1) + 1, 0, n);
                                shallowAll += sh;
                            }
                        continue;
                    }
                    ++caveChunks;
                    world::caveAir(tp, pos, hm, air2);
                    if (std::memcmp(air, air2, sizeof(air)) != 0) repeat = false;
                    const int32_t y0 = cy * world::CHUNK;
                    const auto solid = [&](int x, int y, int z) {   // x, y, z в [-1..32]
                        return y0 + y <= hm.h[z + 1][x + 1] && !(air[z + 1][x + 1] >> (y + 1) & 1);
                    };
                    for (int z = 0; z < world::CHUNK; ++z)
                        for (int x = 0; x < world::CHUNK; ++x) {
                            const int32_t top = hm.h[z + 1][x + 1];
                            int run = 0;
                            for (int y = 0; y < world::CHUNK; ++y) {
                                if (y0 + y > top) break;
                                const bool open = !solid(x, y, z);
                                run = open ? run + 1 : 0;
                                if (run > bestRun && top - (y0 + y) > 8 * world::VOXELS_PER_BLOCK) {
                                    bestRun = run;
                                    camAt[0] = pos.x * world::CHUNK + x; camAt[1] = y0 + y - run / 2; camAt[2] = pos.z * world::CHUNK + z;
                                }
                                ++allVox; airVox += open;
                                if (top - (y0 + y) < tp.caves.tunnelCover) { ++shallowAll; shallowAir += open; }
                            }
                        }
                    if (checked < 12) {
                        int64_t faces = 0;
                        for (int z = 0; z < world::CHUNK; ++z)
                            for (int y = 0; y < world::CHUNK; ++y)
                                for (int x = 0; x < world::CHUNK; ++x) {
                                    if (!solid(x, y, z)) continue;
                                    faces += !solid(x + 1, y, z) + !solid(x - 1, y, z) + !solid(x, y + 1, z) + !solid(x, y - 1, z) +
                                             !solid(x, y, z + 1) + !solid(x, y, z - 1);
                                }
                        world::buildMesh(pos, hm, mesh, ms, air);
                        int64_t area = 0;
                        for (size_t qi = 0; qi + 3 < mesh.size(); qi += 4) {
                            const auto v = [&](size_t i, int sh) { return int((mesh[qi + i] >> sh) & 63); };
                            const int dx = std::abs(v(2, 0) - v(0, 0)), dy = std::abs(v(2, 6) - v(0, 6)), dz = std::abs(v(2, 12) - v(0, 12));
                            area += int64_t(std::max(dx, 1) * std::max(dy, 1) * std::max(dz, 1));
                        }
                        if (area != faces) exact = false;
                        ++checked;
                    }
                }
            }
        std::printf("пещеры: кусков с воздухом %d, сеток проверено %d; воздуха под землёй %.1f%%, в верхних 4 блоках %.2f%%\n",
                    caveChunks, checked, 100.0 * airVox / std::max<int64_t>(allVox, 1), 100.0 * shallowAir / std::max<int64_t>(shallowAll, 1));
        std::printf("пещера для снимка: блок (%.1f, %.1f, %.1f), проём %d вокселей\n", camAt[0] / 16.0, camAt[1] / 16.0, camAt[2] / 16.0, bestRun);
        CHECK(caveChunks > 0, "под землёй есть пещеры");
        CHECK(repeat, "пещеры из одного зерна дважды - одинаковы");
        CHECK(exact, "сетка с пещерами: площадь прямоугольников = видимые грани в лоб (с каймой)");
        CHECK(shallowAll == 0 || shallowAir * 20 < shallowAll, "у поверхности пещер мало (только входы)");
    }

    // ─── Р29: реки и озёра ───────────────────────────────────────────────────────────────
    {
        world::TerrainParams tp;
        // сток: только вниз - у каждого звена сглаженная высота соседа строго ниже, уровень воды не растёт
        bool downhill = true, levels = true;
        int rivers = 0, lakes = 0, maxArea = 0;
        for (int32_t j = -12; j < 12; ++j)
            for (int32_t i = -12; i < 12; ++i) {
                const world::RiverNode n = world::riverNode(tp, i, j);
                if (n.sea) continue;
                maxArea = std::max(maxArea, n.area);
                if (n.down < 0) { if (n.area >= 3) ++lakes; continue; }
                const world::RiverNode t = world::riverNode(tp, n.ti, n.tj);   // Р37: сосед или выход котловины
                if (t.smooth >= n.smooth) downhill = false;
                if (!t.sea && t.down >= 0 && t.level > n.level) levels = false;
                if (n.area >= 3) ++rivers;
            }
        std::printf("реки: узлов с рекой %d, озёр в котловинах %d, наибольший водосбор %d узлов\n", rivers, lakes, maxArea);
        CHECK(downhill, "реки текут только вниз (сглаженная высота соседа строго ниже)");
        CHECK(levels, "уровень воды вниз по течению не растёт");
        CHECK(rivers > 0, "на суше есть реки");
        // на суше есть вода выше уровня моря (реки, озёра)
        int wetLand = 0;
        for (int32_t z = -200000; z < 200000; z += 1777)
            for (int32_t x = -200000; x < 200000; x += 1999) {
                const world::Relief r = world::reliefAt(tp, x, z);
                if (r.water > r.height && r.water > tp.seaLevel - 1) ++wetLand;
            }
        std::printf("реки: точек выборки с водой выше моря %d\n", wetLand);
        CHECK(wetLand > 0, "вода рек и озёр выше уровня моря - есть");

        // Р32: русло проложено низом, уровень - по нему же: вода рек не выше исходной земли (рябь - до 2 блоков),
        // река не поднимает землю под себя; у воды нет сухого соседа ниже её больше чем на 2 блока (столб воды)
        constexpr int N = 256, STEP = 4 * 16;   // квадрат 1024×1024 блока у точки старта, шаг 4 блока
        const int32_t ox = -12450 * 16 - N / 2 * STEP, oz = 23560 * 16 - N / 2 * STEP;
        std::vector<int32_t> H(N * N), W(N * N), H0(N * N);
        int channel = 0, above = 0, raised = 0, columns = 0;
        for (int tj = 0; tj < N; tj += 64)
            for (int ti = 0; ti < N; ti += 64) {
                world::RiverSet rs;
                rs.build(tp, ox + ti * STEP, oz + tj * STEP, ox + (ti + 64) * STEP, oz + (tj + 64) * STEP);
                for (int j = tj; j < tj + 64; ++j)
                    for (int i = ti; i < ti + 64; ++i) {
                        const int32_t x = ox + i * STEP, z = oz + j * STEP;
                        const world::SlowFields s = world::slowAt(tp, x, z);
                        const world::Relief b = world::baseReliefAt(tp, x, z, s), r = world::reliefAt(tp, x, z, s, 0, &rs);
                        H[j * N + i] = r.height;
                        W[j * N + i] = r.water;
                        H0[j * N + i] = b.height;
                        if (r.height > b.height) ++raised;
                        if (r.water > r.height && b.height >= tp.seaLevel) {
                            ++channel;
                            if (r.water - b.height > 2 * 16) ++above;
                        }
                    }
            }
        for (int j = 1; j + 1 < N; ++j)
            for (int i = 1; i + 1 < N; ++i) {
                const int a = j * N + i;
                if (W[a] <= H[a]) continue;
                for (int nb : {a - 1, a + 1, a - N, a + N})
                    if (W[nb] <= H[nb] && W[a] - std::max(H[a], H[nb]) > 2 * 16) ++columns;
            }
        std::printf("реки (Р32): точек русла %d, вода выше исходной земли > 2 бл: %d, земля поднята: %d, столбов воды: %d\n",
                    channel, above, raised, columns);
        CHECK(channel > 0, "в квадрате у старта есть реки");
        CHECK(above == 0, "вода рек не выше исходной земли больше чем на 2 блока");
        CHECK(raised == 0, "река не поднимает землю под себя");
        CHECK(columns == 0, "у воды нет сухого соседа ниже её больше чем на 2 блока (столбов воды)");
    }

    // ─── Р20: дальние уровни ──────────────────────────────────────────────────────────────
    {
        world::TerrainParams tp;
        // плитка уровня 0: шаг 1 блок, отброшенных октав нет - высоты узлов = высоты столбцов вокселей
        world::TileMesh t0;
        world::buildTile(tp, {0, -3, 5}, t0);
        bool same0 = true;
        for (int j = 0; j < world::TILE_VERTS; ++j)
            for (int i = 0; i < world::TILE_VERTS; ++i) {
                const int32_t h = int16_t(t0.data[(j * world::TILE_VERTS + i) * world::TILE_WORDS] & 0xFFFF);
                const int32_t x = (-3 * 32 + i) * 16, z = (5 * 32 + j) * 16;
                if (h != world::reliefAt(tp, x, z).height) same0 = false;
            }
        CHECK(same0, "плитка уровня 0: высоты узлов = высоты столбцов вокселей");

        // ГЛАВНОЕ: сползшая плитка совпадает с родителем. «Высота уровня L+1» узла = высота той же точки в
        // плитке уровня L+1; то же для наклона и покрова. Иначе на смене уровней - щель или скачок цвета.
        bool morphOk = true;
        int checked = 0;
        const int keys[][3] = {{0, 7, -3}, {1, -2, 9}, {2, 3, 3}, {3, -1, -1}, {4, 0, 1}, {5, -1, 0}, {1, 70, -61}, {0, -31, 54}};
        for (const auto& kk : keys) {
            const world::TileKey k{kk[0], kk[1], kk[2]};
            const world::TileKey parent{k.level + 1, k.tx >> 1, k.tz >> 1};
            world::TileMesh child, par;
            world::buildTile(tp, k, child);
            world::buildTile(tp, parent, par);
            for (int j = 0; j < world::TILE_VERTS; ++j)
                for (int i = 0; i < world::TILE_VERTS; ++i) {
                    const int pi = ((k.tx & 1) * world::TILE + (i & ~1)) / 2, pj = ((k.tz & 1) * world::TILE + (j & ~1)) / 2;
                    const uint32_t* c = &child.data[(j * world::TILE_VERTS + i) * world::TILE_WORDS];
                    const uint32_t* pv = &par.data[(pj * world::TILE_VERTS + pi) * world::TILE_WORDS];
                    if ((c[0] >> 16) != (pv[0] & 0xFFFF) || c[2] != pv[1] || ((c[3] >> 8) & 0xFF) != (pv[3] & 0xFF) ||
                        (c[4] >> 16) != (pv[4] & 0xFFFF))
                        morphOk = false;
                    ++checked;
                }
        }
        std::printf("морфинг: проверено %d узлов на уровнях 0..5\n", checked);
        CHECK(morphOk, "сползший узел = узел плитки уровня выше: высота, наклон, покров, вода (без щелей)");

        world::TileMesh t1;
        world::buildTile(tp, {3, -1, -1}, t1);
        world::TileMesh t2;
        world::buildTile(tp, {3, -1, -1}, t2);
        CHECK(t1.data == t2.data, "плитка из одного зерна дважды - одинакова");

        // выбор плиток: каждая точка видимости вне зоны вокселей покрыта ровно одной четвертью; соседние
        // точки - уровни различаются не больше чем на 1 (условие CDLOD)
        world::LodSettings ls;
        ls.drawDistance = 4096;
        ls.holeMinX = -128; ls.holeMinZ = -128; ls.holeMaxX = 128; ls.holeMaxZ = 128;
        const double cx = 37.5, cz = -12.25;
        std::vector<world::TileDraw> sel;
        world::selectTiles(ls, cx, cz, sel);
        constexpr int G = 256, STEP = 32;   // сетка проверки 256×256 точек через 32 блока
        std::vector<int> cover(G * G, 0), level(G * G, -1);
        for (const world::TileDraw& d : sel) {
            const int64_t size = world::tileBlocks(d.key.level), half = size / 2;
            for (int qd = 0; qd < 4; ++qd) {
                if (!(d.quadrants >> qd & 1)) continue;
                const int64_t x0 = int64_t(d.key.tx) * size + (qd & 1) * half, z0 = int64_t(d.key.tz) * size + (qd >> 1) * half;
                for (int gz = 0; gz < G; ++gz)
                    for (int gx = 0; gx < G; ++gx) {
                        const double px = (gx - G / 2) * STEP + 0.5, pz = (gz - G / 2) * STEP + 0.5;
                        if (px >= x0 && px < x0 + half && pz >= z0 && pz < z0 + half) { ++cover[gz * G + gx]; level[gz * G + gx] = d.key.level; }
                    }
            }
        }
        bool once = true, neighbours = true;
        for (int gz = 0; gz < G; ++gz)
            for (int gx = 0; gx < G; ++gx) {
                const double px = (gx - G / 2) * STEP + 0.5, pz = (gz - G / 2) * STEP + 0.5;
                const bool inHole = px > -128 && px < 128 && pz > -128 && pz < 128;
                const double d = std::hypot(px - cx, pz - cz);
                const int n = cover[gz * G + gx];
                if (d < ls.drawDistance - 64 && !inHole && n != 1) once = false;
                if (n > 1) once = false;
                if (gx > 0 && level[gz * G + gx] >= 0 && level[gz * G + gx - 1] >= 0 && std::abs(level[gz * G + gx] - level[gz * G + gx - 1]) > 1)
                    neighbours = false;
            }
        std::printf("выбор плиток: %zu плиток при видимости %d блоков\n", sel.size(), ls.drawDistance);
        CHECK(once, "каждая точка видимости вне вокселей покрыта ровно одной плиткой");
        CHECK(neighbours, "соседние уровни различаются не больше чем на 1");
    }

    std::printf(g_fail ? "\nНЕ ГОДНО: %d\n" : "\nВСЁ ГОДНО\n", g_fail);
    return g_fail ? 1 : 0;
}
