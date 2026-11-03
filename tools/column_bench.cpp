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
        for (int x = 0; x < world::CHUNK; x += 8) seq.push_back(hm.rock.id[0]);
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

int main(int argc, char** argv) {
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
