#include "world/spread.h"

#include <algorithm>
#include <climits>
#include <queue>

#include "world/chunk.h"

namespace world {
namespace {
constexpr int DX[4] = {1, -1, 0, 0}, DZ[4] = {0, 0, 1, -1};
constexpr int32_t B = VOXELS_PER_BLOCK;
constexpr int MAX_LEVEL = 7;   // дальше 7 блоков от источника текущая вода не идёт
constexpr int SEEK = 4;        // понижение ищется не дальше 4 блоков
}  // namespace

void spreadWater(int n, const std::vector<int32_t>& ground, const std::vector<int32_t>& source, std::vector<int32_t>& outTop) {
    const size_t N = size_t(n) * n;
    outTop = source;
    // у столбца с водой: поверхность, с которой она растекается дальше (у падающего столба - его низ), и уровень
    std::vector<int32_t> flowFrom(N, INT32_MIN);
    std::vector<int> level(N, INT_MAX);
    struct Item { int level; int32_t from; int k; };
    const auto worse = [](const Item& a, const Item& b) { return a.level != b.level ? a.level > b.level : a.from < b.from; };
    std::priority_queue<Item, std::vector<Item>, decltype(worse)> q(worse);
    for (size_t k = 0; k < N; ++k)
        if (source[k] != INT32_MIN && source[k] > ground[k]) {
            level[k] = 0;
            flowFrom[k] = source[k];
            q.push({0, source[k], int(k)});
        }
    // понижение: соседняя земля ниже поверхности больше чем на блок - туда вода падает
    const auto isDrop = [&](int k, int32_t surface) { return ground[size_t(k)] < surface - B; };
    // проходимо: земля ниже поверхности
    const auto passable = [&](int k, int32_t surface) { return ground[size_t(k)] < surface - 1; };
    while (!q.empty()) {
        const Item it = q.top();
        q.pop();
        const int k = it.k;
        if (it.level != level[size_t(k)] || it.from != flowFrom[size_t(k)]) continue;
        const int x = k % n, z = k / n;
        const int32_t S = it.from;
        // куда течь: соседи, где можно упасть, - сразу; иначе - по ровному в стороны ближайшего понижения
        int dist[4];
        bool anyDropNear = false;
        for (int d = 0; d < 4; ++d) {
            dist[d] = INT_MAX;
            const int nx = x + DX[d], nz = z + DZ[d];
            if (nx < 0 || nz < 0 || nx >= n || nz >= n) continue;
            const int nk = nz * n + nx;
            if (!passable(nk, S)) continue;
            if (isDrop(nk, S)) { dist[d] = 0; anyDropNear = true; continue; }
            // поиск понижения до SEEK блоков в эту сторону (по проходимым столбцам)
            std::vector<std::pair<int, int>> front{{nk, 1}};
            std::vector<int> seen{k, nk};
            for (size_t f = 0; f < front.size() && dist[d] == INT_MAX; ++f) {
                const auto [ck, cd] = front[f];
                if (cd >= SEEK) continue;
                const int cx = ck % n, cz = ck / n;
                for (int e = 0; e < 4; ++e) {
                    const int ex = cx + DX[e], ez = cz + DZ[e];
                    if (ex < 0 || ez < 0 || ex >= n || ez >= n) continue;
                    const int ek = ez * n + ex;
                    if (std::find(seen.begin(), seen.end(), ek) != seen.end() || !passable(ek, S)) continue;
                    seen.push_back(ek);
                    if (isDrop(ek, S)) { dist[d] = cd + 1; break; }
                    front.push_back({ek, cd + 1});
                }
            }
        }
        int best = INT_MAX;
        for (int d = 0; d < 4; ++d) best = std::min(best, dist[d]);
        (void)anyDropNear;
        for (int d = 0; d < 4; ++d) {
            const int nx = x + DX[d], nz = z + DZ[d];
            if (nx < 0 || nz < 0 || nx >= n || nz >= n) continue;
            const int nk = nz * n + nx;
            if (!passable(nk, S)) continue;
            // по ровному - только к ближайшему понижению (если оно есть в пределах SEEK)
            if (best != INT_MAX && dist[d] != best) continue;
            int nl;
            int32_t top, from;
            if (isDrop(nk, S)) {
                // падает: столб воды от края уступа до земли, внизу растекается с уровня 1
                nl = 1;
                top = S - 1;
                from = ground[size_t(nk)] + (B * (8 - nl)) / 8;
            } else {
                nl = it.level + 1;
                if (nl > MAX_LEVEL) continue;
                const int32_t bed = ground[size_t(nk)];
                from = std::min<int32_t>(S - B / 8, bed + (B * (8 - nl)) / 8);
                if (from <= bed) continue;
                top = from;
            }
            if (nl > level[size_t(nk)] || (nl == level[size_t(nk)] && from <= flowFrom[size_t(nk)])) continue;
            if (source[size_t(nk)] != INT32_MIN && source[size_t(nk)] > ground[size_t(nk)]) continue;   // источник не трогаем
            level[size_t(nk)] = nl;
            flowFrom[size_t(nk)] = from;
            outTop[size_t(nk)] = std::max(outTop[size_t(nk)] == INT32_MIN ? INT32_MIN : outTop[size_t(nk)], top);
            q.push({nl, from, nk});
        }
    }
}

}  // namespace world
