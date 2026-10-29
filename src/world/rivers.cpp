#include "world/rivers.h"

#include <algorithm>
#include <unordered_map>

#include "gen/hash.h"
#include "gen/noise.h"

namespace world {
namespace {

constexpr int64_t CELL = int64_t(1) << RIVER_CELL_LOG2;
constexpr int WINDOW = 8;                                   // окно водосбора 17×17 узлов
constexpr int CHAIN = 16;                                   // не дальше 16 звеньев
constexpr int32_t AREA_MIN = 3;                             // река - с водосбора от 3 узлов
constexpr int32_t WARP = 250 * VOXELS_PER_BLOCK;            // извилистость: сдвиг точки запроса
constexpr int32_t VALLEY_CAP = 400 * VOXELS_PER_BLOCK;      // долина по глубине прорези - не шире на столько
constexpr int SMOOTH_CUT = 15;                              // сглаженная высота - без форм мельче 2048 блоков
// уклон к морю: потенциал стока = сглаженная высота + материковость · TILT; материковость падает к океану на
// ~10 км - мелкие впадины не ловят сток, реки тянутся к побережью, хребты (сотни блоков) по-прежнему главнее
constexpr int64_t TILT = 200 * VOXELS_PER_BLOCK;
constexpr int GROUND_CUT = 9;                               // земля вдоль реки - без ряби мельче 32 блоков
constexpr int DX[8] = {1, 1, 0, -1, -1, -1, 0, 1}, DZ[8] = {0, 1, 1, 1, 0, -1, -1, -1};

struct Info {
    int64_t px, pz;
    int32_t smooth;     // сглаженная высота
    int32_t pot;        // потенциал стока: сглаженная высота + уклон к морю
    bool sea;
    int down = -2;      // -2: ещё не считали
    int32_t area = -1;  // -1: ещё не считали
    bool leveled = false;
    int32_t level = 0;              // Р30: уровень воды в узле
    std::vector<int32_t> run;       // Р30: уровень вдоль вытекающего звена (пусто - ещё не считали)
};

// Узлы - в кэше потока: значения - чистые функции координат и зерна, порядок счёта на них не влияет.
// (Ссылки на элементы unordered_map при росте таблицы не портятся.)
struct Cache {
    std::unordered_map<uint64_t, Info> nodes;
    const TerrainParams* params = nullptr;
    uint32_t seed = 0;
};
thread_local Cache g_cache;

uint64_t nodeKey(int32_t i, int32_t j) { return uint64_t(uint32_t(i)) << 32 | uint32_t(j); }

Info& info(const TerrainParams& p, int32_t i, int32_t j) {
    auto [it, fresh] = g_cache.nodes.try_emplace(nodeKey(i, j));
    Info& n = it->second;
    if (!fresh) return n;
    const uint32_t h = gen::hash2(i, j, p.seed ^ 0x51A7E5u);
    const int64_t jitter = CELL * 35 / 100;
    n.px = int64_t(i) * CELL + CELL / 2 + (int64_t(h & 0xFFFF) - 32768) * jitter / 32768;
    n.pz = int64_t(j) * CELL + CELL / 2 + (int64_t(h >> 16) - 32768) * jitter / 32768;
    const SlowFields slow = slowAt(p, int32_t(n.px), int32_t(n.pz), SMOOTH_CUT);
    const Relief r = baseReliefAt(p, int32_t(n.px), int32_t(n.pz), slow, SMOOTH_CUT);
    n.smooth = r.height;
    n.pot = r.height + int32_t((int64_t(slow.continent) * TILT) >> fx::SHIFT);
    n.sea = r.height < p.seaLevel;
    return n;
}

int down(const TerrainParams& p, int32_t i, int32_t j) {
    Info& n = info(p, i, j);
    if (n.down != -2) return n.down;
    int best = -1;
    int32_t lowest = n.pot;
    if (!n.sea)
        for (int d = 0; d < 8; ++d) {
            const int32_t s = info(p, i + DX[d], j + DZ[d]).pot;
            if (s < lowest) { lowest = s; best = d; }   // строго ниже по потенциалу - иначе котловина
        }
    info(p, i, j).down = best;
    return best;
}

int32_t area(const TerrainParams& p, int32_t i, int32_t j) {
    if (int32_t a = info(p, i, j).area; a >= 0) return a;
    int32_t count = 0;
    for (int dj = -WINDOW; dj <= WINDOW; ++dj)
        for (int di = -WINDOW; di <= WINDOW; ++di) {
            int32_t ci = i + di, cj = j + dj;
            for (int step = 0; step <= CHAIN; ++step) {
                if (ci == i && cj == j) { ++count; break; }
                const int d = down(p, ci, cj);
                if (d < 0) break;
                ci += DX[d];
                cj += DZ[d];
            }
        }
    info(p, i, j).area = count;
    return count;
}

// земля на линии реки минус блок (исходный рельеф без ряби мельче 32 блоков)
int32_t groundLine(const TerrainParams& p, int64_t x, int64_t z) {
    const SlowFields slow = slowAt(p, int32_t(x), int32_t(z), GROUND_CUT);
    return baseReliefAt(p, int32_t(x), int32_t(z), slow, GROUND_CUT).height - VOXELS_PER_BLOCK;
}

const std::vector<int32_t>& outRun(const TerrainParams& p, int32_t i, int32_t j);

// Р30: уровень воды в узле - минимум земли в нём и концов всех впадающих звеньев (притоков); по дереву стока
// вверх до истоков (потенциал строго растёт вверх - рекурсия конечна)
int32_t level(const TerrainParams& p, int32_t i, int32_t j) {
    if (Info& n = info(p, i, j); n.leveled) return n.level;
    const Info& n = info(p, i, j);
    int32_t lv = groundLine(p, n.px, n.pz);
    for (int d = 0; d < 8; ++d) {
        const int32_t ci = i + DX[d], cj = j + DZ[d];
        const int cd = down(p, ci, cj);
        if (cd >= 0 && ci + DX[cd] == i && cj + DZ[cd] == j) lv = std::min(lv, outRun(p, ci, cj).back());
    }
    Info& m = info(p, i, j);
    m.level = std::max(lv, p.seaLevel - 1);
    m.leveled = true;
    return m.level;
}

// Р30: уровень вдоль вытекающего звена - бегущий минимум земли от уровня в узле; точки через ~RUN_STEP
const std::vector<int32_t>& outRun(const TerrainParams& p, int32_t i, int32_t j) {
    if (!info(p, i, j).run.empty()) return info(p, i, j).run;
    const int32_t start = level(p, i, j);
    const int d = down(p, i, j);
    std::vector<int32_t> run;
    if (d < 0) {
        run.push_back(start);
    } else {
        const Info& a = info(p, i, j);
        const Info& b = info(p, i + DX[d], j + DZ[d]);
        const int64_t ex = b.px - a.px, ez = b.pz - a.pz;
        const int n = int(std::max<int64_t>(1, (std::abs(ex) + std::abs(ez)) / RiverSet::RUN_STEP));
        int32_t cur = start;
        run.push_back(cur);
        for (int k = 1; k <= n; ++k) {
            cur = std::min(cur, groundLine(p, a.px + ex * k / n, a.pz + ez * k / n));
            run.push_back(std::max(cur, p.seaLevel - 1));
        }
    }
    Info& m = info(p, i, j);
    m.run = std::move(run);
    return m.run;
}

uint64_t isqrt(uint64_t v) {
    uint64_t r = 0, bit = uint64_t(1) << 62;
    while (bit > v) bit >>= 2;
    while (bit) {
        if (v >= r + bit) { v -= r + bit; r = (r >> 1) + bit; }
        else r >>= 1;
        bit >>= 2;
    }
    return r;
}

// шум вдоль реки в [-1, 1] (у каждого свойства - своё зерно)
inline fx::q16 vary(const TerrainParams& p, int32_t x, int32_t z, int cellLog2, uint32_t salt) {
    return gen::simplex2(x, z, cellLog2, p.seed ^ salt);
}

}  // namespace

void riverNeighbor(int32_t i, int32_t j, int dir, int32_t& ni, int32_t& nj) {
    ni = i + DX[dir];
    nj = j + DZ[dir];
}

RiverNode riverNode(const TerrainParams& p, int32_t i, int32_t j) {
    if (g_cache.params != &p || g_cache.seed != p.seed) {
        g_cache.nodes.clear();
        g_cache.params = &p;
        g_cache.seed = p.seed;
    }
    const int d = down(p, i, j);
    const int32_t a = info(p, i, j).sea ? 0 : area(p, i, j);
    const int32_t lv = level(p, i, j);
    const int32_t end = outRun(p, i, j).back();
    const Info& n = info(p, i, j);
    return {n.px, n.pz, n.pot, n.sea, d, a, lv, end};
}

void RiverSet::build(const TerrainParams& p, int32_t x0, int32_t z0, int32_t x1, int32_t z1) {
    if (g_cache.params != &p || g_cache.seed != p.seed || g_cache.nodes.size() > 400000) {
        g_cache.nodes.clear();
        g_cache.params = &p;
        g_cache.seed = p.seed;
    }
    segs.clear();
    const int64_t margin = 2 * CELL;   // звено, долина и извилистость - не дальше двух клеток
    const auto cell = [](int64_t v) { return int32_t(v >= 0 ? v / CELL : -((-v + CELL - 1) / CELL)); };
    for (int32_t j = cell(z0 - margin); j <= cell(z1 + margin); ++j)
        for (int32_t i = cell(x0 - margin); i <= cell(x1 + margin); ++i) {
            if (info(p, i, j).sea) continue;
            const int32_t a = area(p, i, j);
            if (a < AREA_MIN) continue;
            const int64_t s = int64_t(isqrt(uint64_t(a) << 16));   // √A · 256
            const int d = down(p, i, j);
            Seg g;
            g.ax = info(p, i, j).px;
            g.az = info(p, i, j).pz;
            if (d >= 0) {   // река к соседу ниже
                g.bx = info(p, i + DX[d], j + DZ[d]).px;
                g.bz = info(p, i + DX[d], j + DZ[d]).pz;
                g.half = int32_t(24 + s * 14 / 256);
                g.depth = int32_t(16 + s * 6 / 256);
                g.valley = g.half + int32_t(384 + s * 80 / 256);
            } else {        // котловина - озеро
                g.bx = g.ax;
                g.bz = g.az;
                g.half = int32_t(256 + s * 64 / 256);
                g.depth = int32_t(32 + s * 5 / 256);
                g.valley = g.half + 640;
            }
            // задевает ли прямоугольник (с долиной, извилистостью и разбросом ширины)
            const int64_t r = 2 * g.valley + VALLEY_CAP + 2 * WARP;
            if (std::max(g.ax, g.bx) + r < x0 || std::min(g.ax, g.bx) - r > x1 || std::max(g.az, g.bz) + r < z0 ||
                std::min(g.az, g.bz) - r > z1)
                continue;
            const int64_t ex = g.bx - g.ax, ez = g.bz - g.az;
            g.len = int64_t(isqrt(uint64_t(ex * ex + ez * ez)));
            g.run = outRun(p, i, j);
            if (d < 0) {
                // озеро: уровень не выше самой низкой точки кольца берега - вода не переливается стеной
                int32_t rim = INT32_MAX;
                static constexpr int RX[8] = {2, 1, 0, -1, -2, -1, 0, 1}, RZ[8] = {0, 1, 2, 1, 0, -1, -2, -1};
                for (int k = 0; k < 8; ++k)
                    rim = std::min(rim, groundLine(p, g.ax + RX[k] * g.half / 2, g.az + RZ[k] * g.half / 2));
                g.run = {std::max(std::min(g.run[0], rim), p.seaLevel - 1)};
            }
            segs.push_back(std::move(g));
        }
}

void RiverSet::apply(const TerrainParams& p, int32_t x, int32_t z, int32_t& h, int32_t& water, bool& shore) const {
    if (segs.empty()) return;
    // Р30: разнообразие вдоль реки - медленные шумы в точке (непрерывны - шва нет)
    const fx::q16 nWidth = vary(p, x, z, 12, 0x1D7u), nDepth = vary(p, x, z, 12, 0x2E9u);
    const fx::q16 nPlain = vary(p, x, z, 13, 0x3F1u), nBank = vary(p, x, z, 13, 0x4A3u), nWarp = vary(p, x, z, 15, 0x5B7u);
    const fx::q16 kWidth = fx::ONE * 11 / 10 + nWidth / 2;            // ширина русла ×0.6..1.6
    const fx::q16 kDepth = fx::ONE + nDepth * 3 / 10;                 // глубина ×0.7..1.3
    const fx::q16 kPlain = fx::ONE * 7 / 4 + nPlain * 5 / 4;          // ширина поймы ×0.5..3 (в ширинах русла)
    // стенки долины 1:2..1:8 - заложение bankRun = 5 + 3·шум (Q16, непрерывно: целый делитель прыгал бы
    // на переходе 4 -> 5 - разрыв стенок на сотни вокселей)
    const int64_t bankRun = 5 * int64_t(fx::ONE) + 3 * int64_t(nBank);
    const fx::q16 kWarp = fx::ONE * 9 / 10 + nWarp * 6 / 10;          // извилистость ×0.3..1.5
    // извилистость: точку запроса сдвигаем медленным шумом - русла петляют (сам рельеф этим не искажается)
    const int64_t warp = (int64_t(WARP) * kWarp) >> fx::SHIFT;
    const int64_t qx = x + ((int64_t(gen::simplex2(x, z, 14, p.seed ^ 0x3A11Du)) * warp) >> fx::SHIFT);
    const int64_t qz = z + ((int64_t(gen::simplex2(x, z, 14, p.seed ^ 0x7B23Fu)) * warp) >> fx::SHIFT);
    const int32_t h0 = h, water0 = water;   // water0 - вода до рек (море)
    int32_t levee = INT32_MIN;   // вал: земля не ниже воды у берега (если точка не в русле и не в море)
    bool channel = false;
    for (const Seg& g : segs) {
        // ближайшая точка звена: t - доля пути (Q16)
        const int64_t ex = g.bx - g.ax, ez = g.bz - g.az, px = qx - g.ax, pz = qz - g.az;
        const int64_t len2 = ex * ex + ez * ez;
        int64_t t = 0;
        if (len2 > 0) t = std::clamp<int64_t>(((px * ex + pz * ez) << 16) / len2, 0, fx::ONE);
        const int64_t dx = px - ((ex * t) >> 16), dz = pz - ((ez * t) >> 16);
        const int64_t d2 = dx * dx + dz * dz;
        const int64_t reach = 2 * g.valley + VALLEY_CAP;   // дальше любой долины этого звена
        if (d2 >= reach * reach) continue;
        const int32_t d = int32_t(isqrt(uint64_t(d2)));
        // уровень воды: точки бегущего минимума вдоль звена, между ними - линейно (Р30)
        int32_t lw = g.run[0];
        if (g.run.size() > 1) {
            const int64_t n = int64_t(g.run.size()) - 1;
            const int64_t num = t * n;   // Q16 · n
            const int64_t k = std::min<int64_t>(num >> 16, n - 1), f = num - (k << 16);
            lw = g.run[size_t(k)] + int32_t((int64_t(g.run[size_t(k) + 1] - g.run[size_t(k)]) * f) >> 16);
        }
        const int32_t half = std::max<int32_t>(16, int32_t((int64_t(g.half) * (g.len > 0 ? kWidth : fx::ONE)) >> fx::SHIFT));
        const int32_t depth = int32_t((int64_t(g.depth) * kDepth) >> fx::SHIFT);
        const int32_t plain = half + int32_t((int64_t(half) * kPlain) >> fx::SHIFT);
        // профиль долины: в русле - дно чашей; пойма - на блок выше воды; дальше стенки с заложением bankRun
        int32_t profile;
        if (d < half) profile = lw - depth + int32_t(int64_t(depth) * d * d / (int64_t(half) * half));
        else if (d < plain) profile = lw + 1;
        else profile = lw + 1 + int32_t((int64_t(d - plain) << fx::SHIFT) / bankRun);
        // маска: только прорезает, к краю долины - исходный рельеф. Ширина долины - по глубине прорези в этой
        // точке: на равнине - широкая пологая, в горах - V-образная, не шире VALLEY_CAP
        const int32_t radius = int32_t(std::clamp<int64_t>(plain + ((bankRun * std::max(0, h0 - lw)) >> fx::SHIFT),
                                                           g.valley, g.valley + VALLEY_CAP));
        const fx::q16 w = fx::ONE - fx::smoothstep(radius * 6 / 10, radius, d);
        h = std::min(h, h0 + int32_t((int64_t(w) * std::min(0, profile - h0)) >> fx::SHIFT));
        // вал у берега - насыпью: у кромки на воксель выше воды, наружу снижается 1:1, пока не встретит землю
        // (в море у устья вал не нужен - там уже вода не ниже реки)
        if (d >= half && water0 < lw) levee = std::max(levee, lw + 1 - (d - half));
        if (d < half) { water = std::max(water, lw); channel = true; }
        if (d < half + 2 * VOXELS_PER_BLOCK) shore = true;
    }
    if (!channel && levee > h) h = levee;
}

}  // namespace world
