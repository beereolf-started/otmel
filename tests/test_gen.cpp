// Тесты генерации (Р6): то, что проверяется машиной. Главное - воспроизводимость от зерна (Р2, Р16).
#include <cstdio>
#include <cstdlib>

#include "gen/fixed.h"
#include "gen/hash.h"
#include "gen/noise.h"
#include "world/terrain.h"

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

    std::printf(g_fail ? "\nНЕ ГОДНО: %d\n" : "\nВСЁ ГОДНО\n", g_fail);
    return g_fail ? 1 : 0;
}
