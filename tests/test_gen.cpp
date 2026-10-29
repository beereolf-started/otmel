// Тесты генерации (Р6): то, что проверяется машиной. Главное - воспроизводимость от зерна (Р2, Р16).
#include <cstdio>
#include <cstdlib>

#include "gen/fixed.h"
#include "gen/hash.h"
#include "gen/noise.h"
#include "world/bits.h"
#include "world/mesher.h"
#include "world/terrain.h"

#include <algorithm>
#include <array>
#include <chrono>
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

    // воспроизводимость: один и тот же кусок - одна и та же сумма, в том числе на отрицательных координатах
    world::TerrainParams p;
    const uint64_t a = chunkSum(p, 3, 16, -5), b = chunkSum(p, 3, 16, -5);
    CHECK(a == b, "кусок из одного зерна дважды - одинаков");
    world::TerrainParams q = p;
    q.seed = p.seed + 1;
    CHECK(chunkSum(q, 3, 16, -5) != a, "другое зерно - другой кусок");
    // Эта сумма должна совпадать в отладочной и быстрой сборке и на любой машине (Р2, Р16)
    std::printf("сумма куска (3,16,-5) при зерне 1337: %016llx\n", static_cast<unsigned long long>(a));

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
        bool same = true, sameEdited = true;
        int chunks = 0;
        double fastMs = 0, refMs = 0;
        using clk = std::chrono::steady_clock;
        for (int cz = -5; cz < 5; ++cz)
            for (int cx = -5; cx < 5; ++cx) {
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
                    c.edited = true;      // тот же кусок, но маски - из вокселей
                    world::buildMesh(c, hm, fast, ms);
                    if (quadSet(fast) != quadSet(ref)) sameEdited = false;
                    ++chunks;
                }
            }
        CHECK(same, "сетка по формуле = прежняя сетка (те же прямоугольники)");
        CHECK(sameEdited, "сетка из вокселей = прежняя сетка");
        std::printf("сетка: %d кусков; прежняя %.3f мс на кусок, новая %.3f мс на кусок (в %.1f раза быстрее)\n",
                    chunks, refMs / chunks, fastMs / chunks, refMs / std::max(fastMs, 1e-9));
    }

    std::printf(g_fail ? "\nНЕ ГОДНО: %d\n" : "\nВСЁ ГОДНО\n", g_fail);
    return g_fail ? 1 : 0;
}
