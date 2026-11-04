// Замер цены столбца куска (Р77, долг по цене): сколько стоит columnHeights на равнине и в горах.
// Нужен, чтобы перенос медленных полей разреза на узлы сетки можно было доказать числом, а не на глаз.
//
//   otmel_bench            - по 48 столбцов равнины и гор
//   otmel_bench 200        - по 200 столбцов
//
// Печатает мс на столбец куска (32×32 мира) и мкс на столбец мира - вторая мера удобна для плиток:
// у дальней плитки 33×33 узла, и цена разреза в узле решает, может ли плитка знать породу.
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "world/caves.h"
#include "world/rock_table.h"
#include "world/soil_table.h"
#include "world/terrain.h"

using clk = std::chrono::steady_clock;

namespace {

double msOf(const world::TerrainParams& p, const std::vector<std::pair<int32_t, int32_t>>& cols) {
    static world::ColumnHeights hm;
    for (const auto& [cx, cz] : cols) world::columnHeights(p, cx, cz, hm);   // прогрев кэшей и таблиц шумов
    const auto t0 = clk::now();
    for (const auto& [cx, cz] : cols) world::columnHeights(p, cx, cz, hm);
    return std::chrono::duration<double, std::milli>(clk::now() - t0).count() / double(cols.size());
}

/** Верх дна столбца: по нему и судим, суша это или океан (maxH учитывает воду и в океане равен глади). */
int32_t topOf(const world::ColumnHeights& hm) {
    int32_t top = INT32_MIN;
    for (int z = 1; z <= world::CHUNK; ++z)
        for (int x = 1; x <= world::CHUNK; ++x) top = std::max(top, hm.h[z][x]);
    return top;
}

/** Суша, отсортированная по «горности» - разбросу высот внутри столбца. */
std::vector<std::pair<int32_t, int32_t>> pick(const world::TerrainParams& p, int want, bool mountains, int& looked) {
    static world::ColumnHeights hm;
    std::vector<std::pair<int64_t, std::pair<int32_t, int32_t>>> found;
    // Суша в этом мире разведана далеко от начала координат (tests/test_gen.cpp, landChunk) - обходим вокруг неё.
    const int32_t cx0 = -6225, cz0 = 11780;
    const int side = 40, step = 13;            // ±260 столбцов кусков ≈ ±16 тысяч блоков вокруг разведанного места
    for (int k = 0; k < side * side && int(found.size()) < want * 4; ++k) {
        const int32_t cx = cx0 + int32_t((k % side - side / 2) * step), cz = cz0 + int32_t((k / side - side / 2) * step);
        world::columnHeights(p, cx, cz, hm);
        ++looked;
        const int32_t top = topOf(hm);
        if (top <= p.seaLevel + 4 * world::VOXELS_PER_BLOCK) continue;        // океан и отмель - не о них речь
        int32_t low = INT32_MAX;
        for (int z = 1; z <= world::CHUNK; ++z)
            for (int x = 1; x <= world::CHUNK; ++x) low = std::min(low, hm.h[z][x]);
        found.push_back({int64_t(top) - low, {cx, cz}});
    }
    std::sort(found.begin(), found.end(),
              [mountains](const auto& a, const auto& b) { return mountains ? a.first > b.first : a.first < b.first; });
    std::vector<std::pair<int32_t, int32_t>> out;
    for (int i = 0; i < want && i < int(found.size()); ++i) out.push_back(found[size_t(i)].second);
    return out;
}

}  // namespace

/**
 * Разведка пород по прямой: идём на восток и печатаем, какая порода лежит верхним слоем разреза.
 * Нужна, чтобы увидеть, что породы СМЕНЯЮТ ДРУГ ДРУГА через промежуточные (Р78), а не прыгают.
 */
void walkRocks(const world::TerrainParams& p, int32_t cx0, int32_t cz0, int cols) {
    static world::ColumnHeights hm;
    std::vector<int> seq;
    for (int k = 0; k < cols; ++k) {
        world::columnHeights(p, cx0 + k, cz0, hm);
        for (int x = 0; x < world::CHUNK; x += 8) seq.push_back(hm.rock[0][0].id[0]);
    }
    std::printf("разведка пород: %d столбцов кусков на восток от (%d, %d)\n", cols, cx0, cz0);
    int runs = 0;
    size_t i = 0;
    while (i < seq.size()) {
        size_t j = i;
        while (j < seq.size() && seq[j] == seq[i]) ++j;
        if (runs < 40) std::printf("  %-24s %4d шагов\n", world::rockDef(seq[i]).name, int(j - i) * 4);
        ++runs;
        i = j;
    }
    bool met[world::ROCK_COUNT] = {};
    for (int r : seq) met[r] = true;
    int kinds = 0, novel = 0;
    for (int r = 0; r < world::ROCK_COUNT; ++r)
        if (met[r]) { ++kinds; if (r >= world::ROCK_TFG) ++novel; }
    std::printf("  всего участков %d, разных пород %d, из них промежуточных (Р78) %d\n", runs, kinds, novel);
}

/** Цена столбцов слева и справа от места: не дороже ли считать мир в отрицательную сторону. */
void mirror(const world::TerrainParams& p, int32_t cx0, int32_t cz0, int cols) {
    std::vector<std::pair<int32_t, int32_t>> west, east;
    for (int k = 1; k <= cols; ++k) {
        west.push_back({cx0 - k, cz0});
        east.push_back({cx0 + k, cz0});
    }
    const double w = msOf(p, west), e = msOf(p, east);
    std::printf("цена столбца слева и справа от (%d, %d), по %d штук:\n", cx0, cz0, cols);
    std::printf("  на запад: %6.2f мс    на восток: %6.2f мс    разница %+.1f%%\n", w, e, (w / e - 1.0) * 100.0);
}

/**
 * Разбор пещер: какую долю подземелья они занимают и какой высоты ходы. Нужен, чтобы судить о
 * «гигантизме» числом: основой должны быть узкие ходы, а залы - находкой.
 */
void caveStats(const world::TerrainParams& p, int32_t cx0, int32_t cz0, int cols) {
    static world::ColumnHeights hm;
    static uint64_t air[world::CHUNK + 2][world::CHUNK + 2];
    int64_t open = 0, total = 0;
    int64_t runs[6] = {};   // высота хода: 1, 2, 3, 4-6, 7-12, больше
    for (int k = 0; k < cols; ++k) {
        const int32_t cx = cx0 + (k % 8) * 3, cz = cz0 + (k / 8) * 3;
        world::columnHeights(p, cx, cz, hm);
        const int32_t topY = hm.maxH / world::CHUNK;
        for (int32_t cy = topY - 20; cy <= topY; ++cy) {
            if (!world::caveAir(p, world::ChunkPos{cx, cy, cz}, hm, air)) { total += int64_t(world::CHUNK) * world::CHUNK * world::CHUNK; continue; }
            for (int z = 1; z <= world::CHUNK; ++z)
                for (int x = 1; x <= world::CHUNK; ++x) {
                    int run = 0;
                    for (int y = 0; y < world::CHUNK; ++y) {
                        ++total;
                        if ((air[z][x] >> y) & 1) { ++open; ++run; continue; }
                        if (run) {
                            // высота в блоках, с округлением вверх: ход в воксель - это уже «меньше блока»
                            const int b = (run + world::VOXELS_PER_BLOCK - 1) / world::VOXELS_PER_BLOCK;
                            runs[b <= 1 ? 0 : b == 2 ? 1 : b == 3 ? 2 : b <= 6 ? 3 : b <= 12 ? 4 : 5] += 1;
                            run = 0;
                        }
                    }
                }
        }
    }
    // Как часто на местности попадается вход. Точки разносим ШИРОКО: вход - редкость, и мерить его
    // внутри нескольких соседних кусков бессмысленно, там его не будет никогда.
    int64_t mouths = 0, looked = 0;
    const int side = 40, stride = 32;        // 40×40 мест через 32 куска = через 64 блока, охват ~2.5 км
    for (int gz = 0; gz < side; ++gz)
        for (int gx = 0; gx < side; ++gx) {
            const int32_t cx = cx0 + (gx - side / 2) * stride, cz = cz0 + (gz - side / 2) * stride;
            world::columnHeights(p, cx, cz, hm);
            const int32_t wx = cx * world::CHUNK + world::CHUNK / 2, wz = cz * world::CHUNK + world::CHUNK / 2;
            ++looked;
            // настоящее отверстие: прорезаем верхние куски и смотрим, выходит ли пустота к поверхности
            bool holed = false;
            const int32_t topY = hm.maxH / world::CHUNK;
            for (int32_t cy = topY; cy >= topY - 2 && !holed; --cy) {
                if (!world::caveAir(p, world::ChunkPos{cx, cy, cz}, hm, air)) continue;
                for (int z = 1; z <= world::CHUNK && !holed; ++z)
                    for (int x = 1; x <= world::CHUNK && !holed; ++x) {
                        const int32_t top = hm.h[z][x];
                        for (int y = 0; y < world::CHUNK; ++y) {
                            if (!((air[z][x] >> y) & 1)) continue;
                            const int32_t wy = cy * world::CHUNK + y;
                            if (top - wy < world::VOXELS_PER_BLOCK) { holed = true; break; }
                        }
                    }
            }
            if (holed) {
                ++mouths;
                if (mouths <= 5) std::printf("  отверстие: блоки %d %d\n", wx, wz);
            }
        }
    int64_t n = 0;
    for (int64_t r : runs) n += r;
    std::printf("пещеры на %d кусках: пустота занимает %.2f%% подземелья, ходов найдено %lld\n",
                cols, 100.0 * double(open) / double(total ? total : 1), (long long)n);
    std::printf("  отверстие к поверхности найдено в %.2f%% мест (%lld из %lld), места через 64 блока\n",
                100.0 * double(mouths) / double(looked ? looked : 1), (long long)mouths, (long long)looked);
    const char* NAME[6] = {"до 1 блока", "2 блока", "3 блока", "4-6 блоков", "7-12 блоков", "больше 12"};
    for (int i = 0; i < 6; ++i)
        std::printf("  высота %-12s %6.2f%%\n", NAME[i], 100.0 * double(runs[i]) / double(n ? n : 1));
}


/**
 * Разбор места по вокселям: ряд соседних столбцов, и в каждом - где пустота, где земля, где порода.
 * Нужен, чтобы судить о висящих слитках и обвалах числом, а не по картинке.
 */
void probe(const world::TerrainParams& p, int32_t bx, int32_t bz, int cols, bool alongZ = false, int stepVox = world::VOXELS_PER_BLOCK) {
    static world::ColumnHeights hm;
    static uint64_t air[5][world::CHUNK + 2][world::CHUNK + 2];
    const int32_t vx0 = bx * world::VOXELS_PER_BLOCK, vz = bz * world::VOXELS_PER_BLOCK;
    std::printf("разбор у блока (%d, %d), %d столбцов на восток\n", bx, bz, cols);
    for (int k = 0; k < cols; ++k) {
        const int32_t vx = vx0 + (alongZ ? 0 : k * stepVox);
        const int32_t cx = vx >= 0 ? vx / world::CHUNK : -((-vx + world::CHUNK - 1) / world::CHUNK);
        const int32_t vzk = vz + (alongZ ? k * stepVox : 0);
        const int32_t cz = vzk >= 0 ? vzk / world::CHUNK : -((-vzk + world::CHUNK - 1) / world::CHUNK);
        world::columnHeights(p, cx, cz, hm);
        const int lx = int(vx - cx * world::CHUNK), lz = int(vzk - cz * world::CHUNK);
        const int32_t top = hm.h[lz + 1][lx + 1];
        const int32_t cyTop = top >= 0 ? top / world::CHUNK : -((-top + world::CHUNK - 1) / world::CHUNK);
        bool has[5];
        static uint8_t cutCol[world::CHUNK + 2][world::CHUNK + 2];
        for (int j = 0; j < 5; ++j) has[j] = world::caveAir(p, world::ChunkPos{cx, cyTop - j, cz}, hm, air[j]);
        world::soilCut(p, cx, cz, hm, cutCol);
        const int srez = cutCol[lz + 1][lx + 1];
        const world::Material deep = world::rockAt(hm.rock, vx, top - 10 * world::VOXELS_PER_BLOCK, vzk);
        std::printf("  %7.2f бл: верх %6.1f, земли %5.2f/%5.2f, срез %5.2f;", (alongZ ? vzk : vx) / 16.0, top / 16.0,
                    hm.cover[lz][lx].soil / 16.0, hm.soilThick[lz + 1][lx + 1] / 16.0, srez / 16.0);
        std::printf(" вход %+.4f;", world::entranceNoise(p, vx, vzk) / 65536.0);
        {   // Р87: какая здесь почва (тело земли под покровом)
            const world::Material u = hm.cover[lz][lx].under;
            std::printf(" почва %s;", u >= world::SOIL_FIRST && u <= world::SOIL_LAST
                                          ? world::soilDef(int(u) - world::SOIL_FIRST).name
                                          : "-");
        }
        int runs = 0;
        int32_t runFrom = INT32_MIN, prev = INT32_MIN;
        for (int j = 0; j < 5; ++j) {
            if (!has[j]) continue;
            for (int y = world::CHUNK - 1; y >= 0; --y) {
                if (!((air[j][lz + 1][lx + 1] >> (y + 1)) & 1)) continue;
                const int32_t wy = (cyTop - j) * world::CHUNK + y;
                if (wy > top) continue;
                if (prev != wy + 1) {
                    if (runFrom != INT32_MIN && runs < 4)
                        std::printf(" пустота %.1f..%.1f бл;", prev / 16.0, runFrom / 16.0);
                    if (runFrom != INT32_MIN) ++runs;
                    runFrom = wy;
                }
                prev = wy;
            }
        }
        if (runFrom != INT32_MIN && runs < 4) std::printf(" пустота %.1f..%.1f бл;", prev / 16.0, runFrom / 16.0);
        std::printf("\n");
    }
}


/**
 * Поиск столбов: воксельные столбцы, у которых срез земли нулевой, а у всех четырёх соседей
 * заметный. Такой столбец и остаётся стоять столбиком среди срезанной земли.
 */
void pillars(const world::TerrainParams& p, int32_t bx, int32_t bz, int sideBlocks) {
    static world::ColumnHeights hm;
    static uint64_t air[world::CHUNK + 2][world::CHUNK + 2];
    static uint64_t masks[6][world::CHUNK + 2][world::CHUNK + 2];
    static int32_t surf[world::CHUNK + 2][world::CHUNK + 2];
    static uint8_t best[world::CHUNK + 2][world::CHUNK + 2];
    const int32_t cx0 = (bx * world::VOXELS_PER_BLOCK) / world::CHUNK;
    const int32_t cz0 = (bz * world::VOXELS_PER_BLOCK) / world::CHUNK;
    const int side = std::max(2, sideBlocks * world::VOXELS_PER_BLOCK / world::CHUNK);
    int64_t found = 0, looked = 0;
    std::printf("поиск столбов: %d столбцов кусков от блока (%d, %d)\n", side * side, bx, bz);
    for (int j = 0; j < side; ++j)
        for (int i = 0; i < side; ++i) {
            const int32_t cx = cx0 + i, cz = cz0 + j;
            world::columnHeights(p, cx, cz, hm);
            const int32_t cyTop = hm.maxH / world::CHUNK;
            world::soilCut(p, cx, cz, hm, best);
            for (int k = 0; k < 6; ++k) {
                const bool ok = world::caveAir(p, world::ChunkPos{cx, cyTop - k, cz}, hm, air);
                for (int z = 0; z < world::CHUNK + 2; ++z)
                    for (int x = 0; x < world::CHUNK + 2; ++x) masks[k][z][x] = ok ? air[z][x] : 0;
            }
            // верх столбца: высота минус срез земли, а затем вниз, пока воксель помечен пустотой
            for (int z = 0; z < world::CHUNK + 2; ++z)
                for (int x = 0; x < world::CHUNK + 2; ++x) {
                    int32_t wy = hm.h[z][x] - best[z][x];
                    for (int guard = 0; guard < 8 * world::VOXELS_PER_BLOCK; ++guard) {
                        const int32_t cy = wy >= 0 ? wy / world::CHUNK : -((-wy + world::CHUNK - 1) / world::CHUNK);
                        const int k = int(cyTop - cy);
                        if (k < 0 || k >= 6) break;
                        const int yi = int(wy - (cy * world::CHUNK - 1));
                        if (yi < 0 || yi >= world::CHUNK + 2 || !((masks[k][z][x] >> yi) & 1)) break;
                        --wy;
                    }
                    surf[z][x] = wy;
                }
            for (int z = 1; z <= world::CHUNK; ++z)
                for (int x = 1; x <= world::CHUNK; ++x) {
                    ++looked;
                    const int32_t lo4 = std::max(std::max(surf[z][x - 1], surf[z][x + 1]),
                                                 std::max(surf[z - 1][x], surf[z + 1][x]));
                    const int32_t hi4 = std::min(std::min(surf[z][x - 1], surf[z][x + 1]),
                                                 std::min(surf[z - 1][x], surf[z + 1][x]));
                    const bool pillar = surf[z][x] - lo4 >= 2 * world::VOXELS_PER_BLOCK;   // столб
                    const bool pit = hi4 - surf[z][x] >= 2 * world::VOXELS_PER_BLOCK;      // яма
                    if (!pillar && !pit) continue;
                    const int32_t hi = pillar ? lo4 : hi4;
                    ++found;
                    if (found <= 8) {
                        world::Cover cv = hm.cover[std::min(z, world::CHUNK) - 1][std::min(x, world::CHUNK) - 1];
                        cv.soil = int16_t(std::max(0, int(cv.soil) - int(best[z][x])));   // срез уменьшает толщу
                        const world::Material m = world::columnMaterial(hm.h[z][x] - best[z][x], cv, surf[z][x],
                                                                       hm.water[z][x], hm.rock,
                                                                       cx * world::CHUNK + x - 1, cz * world::CHUNK + z - 1);
                        std::printf("  %s: блок (%.2f, %.2f, %.2f), соседи на %.2f, материал %d, "
                                    "срез свой %.2f, соседей %.2f %.2f %.2f %.2f, земли %.2f\n",
                                    pillar ? "столб" : "яма", (cx * world::CHUNK + x - 1) / 16.0, surf[z][x] / 16.0,
                                    (cz * world::CHUNK + z - 1) / 16.0, std::abs(hi - surf[z][x]) / 16.0, int(m),
                                    best[z][x] / 16.0, best[z][x - 1] / 16.0, best[z][x + 1] / 16.0,
                                    best[z - 1][x] / 16.0, best[z + 1][x] / 16.0, cv.soil / 16.0);
                    }
                }
        }
    std::printf("  столбов %lld из %lld столбцов\n", (long long)found, (long long)looked);
}


/** Шов: один и тот же мировой столбец считают два соседних столбца кусков - сходятся ли срезы. */
void seams(const world::TerrainParams& p, int32_t bx, int32_t bz, int sideBlocks) {
    static world::ColumnHeights hmA, hmB;
    static uint8_t cutA[world::CHUNK + 2][world::CHUNK + 2], cutB[world::CHUNK + 2][world::CHUNK + 2];
    const int32_t cx0 = (bx * world::VOXELS_PER_BLOCK) / world::CHUNK;
    const int32_t cz0 = (bz * world::VOXELS_PER_BLOCK) / world::CHUNK;
    const int side = std::max(2, sideBlocks * world::VOXELS_PER_BLOCK / world::CHUNK);
    int64_t bad = 0, looked = 0, worst = 0;
    for (int j = 0; j < side; ++j)
        for (int i = 0; i < side; ++i) {
            const int32_t cx = cx0 + i, cz = cz0 + j;
            world::columnHeights(p, cx, cz, hmA);
            world::soilCut(p, cx, cz, hmA, cutA);
            world::columnHeights(p, cx + 1, cz, hmB);
            world::soilCut(p, cx + 1, cz, hmB, cutB);
            // общий мировой столбец x = (cx+1)*CHUNK
            for (int z = 1; z <= world::CHUNK; ++z) {
                ++looked;
                // у A это индекс CHUNK+1 (мировой x = (cx+1)*CHUNK), у B - индекс 1: кайма сдвинута
                const int a = cutA[z][world::CHUNK + 1], b = cutB[z][1];
                if (a == b) continue;
                ++bad;
                worst = std::max<int64_t>(worst, std::abs(a - b));
                if (bad <= 5)
                    std::printf("  шов: блок (%.2f, %.2f), срез у своего %.2f, у соседа %.2f\n",
                                ((cx + 1) * world::CHUNK - 1) / 16.0, (cz * world::CHUNK + z - 1) / 16.0, a / 16.0, b / 16.0);
            }
        }
    std::printf("швов %lld из %lld столбцов, наибольшее расхождение %.2f бл\n", (long long)bad, (long long)looked, worst / 16.0);
}

/** Где воронка вообще есть: ищем столбцы с ненулевым срезом земли. */
void cuts(const world::TerrainParams& p, int32_t bx, int32_t bz, int sideBlocks) {
    static world::ColumnHeights hm;
    static uint8_t cut[world::CHUNK + 2][world::CHUNK + 2];
    const int32_t cx0 = (bx * world::VOXELS_PER_BLOCK) / world::CHUNK;
    const int32_t cz0 = (bz * world::VOXELS_PER_BLOCK) / world::CHUNK;
    const int side = std::max(2, sideBlocks * world::VOXELS_PER_BLOCK / world::CHUNK);
    int64_t with = 0, looked = 0;
    int shown = 0, worstStep = 0;
    int32_t worstAt[2] = {};
    for (int j = 0; j < side; ++j)
        for (int i = 0; i < side; ++i) {
            const int32_t cx = cx0 + i, cz = cz0 + j;
            world::columnHeights(p, cx, cz, hm);
            world::soilCut(p, cx, cz, hm, cut);
            int best = 0;
            for (int z = 1; z <= world::CHUNK; ++z)
                for (int x = 1; x <= world::CHUNK; ++x) {
                    ++looked;
                    if (cut[z][x]) ++with;
                    best = std::max(best, int(cut[z][x]));
                    // ступенька среза к соседям: мера гладкости воронки. Большая ступенька и есть
                    // отвесная стенка на земле.
                    // ступенька на ПОВЕРХНОСТИ сверх той, что даёт сам рельеф: только она и видна
                    const auto step1 = [&](int z2, int x2) {
                        const int now = std::abs((int(hm.h[z][x]) - int(cut[z][x])) - (int(hm.h[z2][x2]) - int(cut[z2][x2])));
                        const int raw = std::abs(int(hm.h[z][x]) - int(hm.h[z2][x2]));
                        return now - raw;
                    };
                    const int st = std::max(std::max(step1(z, x - 1), step1(z, x + 1)),
                                            std::max(step1(z - 1, x), step1(z + 1, x)));
                    if (st > worstStep) {
                        worstStep = st;
                        worstAt[0] = cx * world::CHUNK + x - 1;
                        worstAt[1] = cz * world::CHUNK + z - 1;
                    }
                }
            if (best >= world::VOXELS_PER_BLOCK && shown < 8) {
                ++shown;
                std::printf("  воронка: блоки %d %d, наибольший срез %.2f бл\n", cx * world::CHUNK / 16,
                            cz * world::CHUNK / 16, best / 16.0);
            }
        }
    if (worstStep > 0) {
        // окрестность худшей ступеньки по вокселям: видно, стенка это или ровный скат
        const int32_t cx = worstAt[0] >= 0 ? worstAt[0] / world::CHUNK : -((-worstAt[0] + world::CHUNK - 1) / world::CHUNK);
        const int32_t cz = worstAt[1] >= 0 ? worstAt[1] / world::CHUNK : -((-worstAt[1] + world::CHUNK - 1) / world::CHUNK);
        world::columnHeights(p, cx, cz, hm);
        world::soilCut(p, cx, cz, hm, cut);
        const int lx = int(worstAt[0] - cx * world::CHUNK) + 1, lz = int(worstAt[1] - cz * world::CHUNK) + 1;
        for (int dz = -3; dz <= 3; ++dz) {
            std::printf("    ");
            for (int dx = -3; dx <= 3; ++dx) {
                const int z = std::clamp(lz + dz, 0, world::CHUNK + 1), x = std::clamp(lx + dx, 0, world::CHUNK + 1);
                std::printf("%5.2f ", cut[z][x] / 16.0);
            }
            std::printf("\n");
        }
    }
    std::printf("  наибольшая ступенька среза между соседями %.2f бл у блока (%d, %d)\n", worstStep / 16.0,
                worstAt[0] / 16, worstAt[1] / 16);
    std::printf("срез есть у %lld столбцов из %lld (%.3f%%)\n", (long long)with, (long long)looked,
                100.0 * double(with) / double(looked ? looked : 1));
}


/**
 * Висящая земля: воксель земли, под которым пустота. Это и навес, и столб, и крышка над пещерой -
 * всё, что запрещено правилом «над пустотой либо камень, либо ничего». Должно быть НОЛЬ.
 */
void hanging(const world::TerrainParams& p, int32_t bx, int32_t bz, int sideBlocks) {
    static world::ColumnHeights hm;
    static uint64_t air[world::CHUNK + 2][world::CHUNK + 2];
    static uint8_t cut[world::CHUNK + 2][world::CHUNK + 2];
    static uint64_t masks[8][world::CHUNK + 2][world::CHUNK + 2];
    const int32_t cx0 = (bx * world::VOXELS_PER_BLOCK) / world::CHUNK;
    const int32_t cz0 = (bz * world::VOXELS_PER_BLOCK) / world::CHUNK;
    const int side = std::max(2, sideBlocks * world::VOXELS_PER_BLOCK / world::CHUNK);
    int64_t bad = 0, looked = 0;
    int shown = 0;
    const auto isSoil = [](world::Material m) {
        return m == world::DIRT || m == world::GRASS || m == world::SAND || m == world::SNOW;
    };
    for (int j = 0; j < side; ++j)
        for (int i = 0; i < side; ++i) {
            const int32_t cx = cx0 + i, cz = cz0 + j;
            world::columnHeights(p, cx, cz, hm);
            world::soilCut(p, cx, cz, hm, cut);
            const int32_t cyTop = hm.maxH / world::CHUNK;
            for (int k = 0; k < 8; ++k) {
                const bool ok = world::caveAir(p, world::ChunkPos{cx, cyTop - k, cz}, hm, air, cut);
                for (int z = 0; z < world::CHUNK + 2; ++z)
                    for (int x = 0; x < world::CHUNK + 2; ++x) masks[k][z][x] = ok ? air[z][x] : 0;
            }
            const auto isAir = [&](int z, int x, int32_t wy) {
                const int32_t cy = wy >= 0 ? wy / world::CHUNK : -((-wy + world::CHUNK - 1) / world::CHUNK);
                const int k = int(cyTop - cy);
                if (k < 0 || k >= 8) return false;
                const int yi = int(wy - (cy * world::CHUNK - 1));
                return yi >= 0 && yi < world::CHUNK + 2 && ((masks[k][z][x] >> yi) & 1) != 0;
            };
            for (int z = 1; z <= world::CHUNK; ++z)
                for (int x = 1; x <= world::CHUNK; ++x) {
                    const int32_t top = hm.h[z][x] - cut[z][x];
                    const world::Cover cv = hm.cover[z - 1][x - 1];
                    const int32_t wx = cx * world::CHUNK + x - 1, wz = cz * world::CHUNK + z - 1;
                    for (int32_t wy = top; wy > top - 8 * world::VOXELS_PER_BLOCK; --wy) {
                        ++looked;
                        if (isAir(z, x, wy)) continue;
                        world::Cover cvEff = cv;
                        cvEff.soil = int16_t(std::max(0, int(cv.soil) - int(cut[z][x])));
                        const world::Material m =
                            world::columnMaterial(top, cvEff, wy, hm.water[z][x], hm.rock, wx, wz);
                        if (!isSoil(m)) continue;
                        if (!isAir(z, x, wy - 1)) continue;
                        ++bad;
                        if (shown < 8) {
                            ++shown;
                            std::printf("  висит: блок (%.2f, %.2f, %.2f), материал %d, верх %.2f, срез %.2f\n",
                                        wx / 16.0, wy / 16.0, wz / 16.0, int(m), top / 16.0, cut[z][x] / 16.0);
                        }
                    }
                }
        }
    std::printf("висящей земли %lld вокселей из %lld проверенных\n", (long long)bad, (long long)looked);
}

int main(int argc, char** argv) {
    if (argc > 3 && std::string(argv[1]) == "--hang") {
        world::TerrainParams tp;
        hanging(tp, atoi(argv[2]), atoi(argv[3]), argc > 4 ? std::max(4, atoi(argv[4])) : 32);
        return 0;
    }
    if (argc > 3 && std::string(argv[1]) == "--cuts") {
        world::TerrainParams tp;
        cuts(tp, atoi(argv[2]), atoi(argv[3]), argc > 4 ? std::max(4, atoi(argv[4])) : 32);
        return 0;
    }
    if (argc > 3 && std::string(argv[1]) == "--seam") {
        world::TerrainParams tp;
        seams(tp, atoi(argv[2]), atoi(argv[3]), argc > 4 ? std::max(4, atoi(argv[4])) : 24);
        return 0;
    }
    if (argc > 3 && std::string(argv[1]) == "--pillars") {
        world::TerrainParams tp;
        pillars(tp, atoi(argv[2]), atoi(argv[3]), argc > 4 ? std::max(4, atoi(argv[4])) : 32);
        return 0;
    }
    if (argc > 3 && std::string(argv[1]) == "--probe") {
        world::TerrainParams tp;
        probe(tp, atoi(argv[2]), atoi(argv[3]), argc > 4 ? std::max(2, atoi(argv[4])) : 24, argc > 5,
              argc > 6 ? std::max(1, atoi(argv[6])) : world::VOXELS_PER_BLOCK);
        return 0;
    }
    if (argc > 1 && std::string(argv[1]) == "--caves") {
        world::TerrainParams tp;
        caveStats(tp, -6225, 11780, argc > 2 ? std::max(4, atoi(argv[2])) : 24);
        return 0;
    }
    if (argc > 1 && std::string(argv[1]) == "--mirror") {
        world::TerrainParams tp;
        mirror(tp, -99600 / 2, 188480 / 2, argc > 2 ? std::max(8, atoi(argv[2])) : 64);
        return 0;
    }
    if (argc > 1 && std::string(argv[1]) == "--rocks") {
        world::TerrainParams tp;
        walkRocks(tp, -6225, 11780, argc > 2 ? std::max(4, atoi(argv[2])) : 60);
        return 0;
    }
    const int want = argc > 1 ? std::max(8, atoi(argv[1])) : 48;
    world::TerrainParams p;

    int looked = 0, looked2 = 0;
    const auto flat = pick(p, want, false, looked);
    const auto hills = pick(p, want, true, looked2);
    if (flat.empty() || hills.empty()) {
        std::printf("суши в обзоре не нашлось (просмотрено столбцов: %d)\n", looked);
        return 1;
    }
    const double flatMs = msOf(p, flat), hillMs = msOf(p, hills);
    const double per = 32.0 * 32.0;   // столбцов мира в столбце куска

    std::printf("цена столбца (в выборке по %d, просмотрено %d)\n", int(flat.size()), looked);
    std::printf("  пологое: %7.2f мс на столбец куска  (%6.1f мкс на столбец мира)\n", flatMs, flatMs * 1000.0 / per);
    std::printf("  горы:    %7.2f мс на столбец куска  (%6.1f мкс на столбец мира)\n", hillMs, hillMs * 1000.0 / per);
    std::printf("  плитка 33x33 узла обошлась бы в %.1f мс (в горах %.1f мс)\n",
                flatMs * 33.0 * 33.0 / per, hillMs * 33.0 * 33.0 / per);
    return 0;
}
