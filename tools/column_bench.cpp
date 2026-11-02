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
#include <vector>

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

int main(int argc, char** argv) {
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
