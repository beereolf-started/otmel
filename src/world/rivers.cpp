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
constexpr int32_t MARGIN = 2 * VOXELS_PER_BLOCK;            // уровень воды ниже сглаженной высоты соседа
constexpr int32_t WARP = 250 * VOXELS_PER_BLOCK;            // извилистость: сдвиг точки запроса
constexpr int32_t VALLEY_CAP = 400 * VOXELS_PER_BLOCK;      // долина по глубине прорези - не шире на столько
constexpr int SMOOTH_CUT = 15;                              // сглаженная высота - без форм мельче 2048 блоков
// уклон к морю: потенциал стока = сглаженная высота + материковость · TILT; материковость падает к океану на
// ~10 км - мелкие впадины не ловят сток, реки тянутся к побережью, хребты (сотни блоков) по-прежнему главнее
constexpr int64_t TILT = 200 * VOXELS_PER_BLOCK;
constexpr int GROUND_CUT = 10;                              // земля вдоль реки - без ряби мельче 64 блоков
constexpr int DX[8] = {1, 1, 0, -1, -1, -1, 0, 1}, DZ[8] = {0, 1, 1, 1, 0, -1, -1, -1};

struct Info {
    int64_t px, pz;
    int32_t smooth;     // сглаженная высота
    int32_t pot;        // потенциал стока: сглаженная высота + уклон к морю
    bool sea;
    int down = -2;      // -2: ещё не считали
    int32_t area = -1;  // -1: ещё не считали
};

// Узлы - в кэше потока: значения - чистые функции координат и зерна, порядок счёта на них не влияет
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

int32_t level(const TerrainParams& p, int32_t i, int32_t j) {
    const int d = down(p, i, j);
    const Info& n = info(p, i, j);
    // расчётный уровень - по потенциалу (вниз по течению только падает); итоговый - ещё и не выше земли на линии
    if (d < 0) return std::max(n.pot - MARGIN, p.seaLevel - 1);   // котловина - озеро; море - уровень моря
    const Info& t = info(p, i + DX[d], j + DZ[d]);
    return t.sea ? p.seaLevel - 1 : std::max(t.pot - MARGIN, p.seaLevel - 1);
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

}  // namespace

void riverNeighbor(int32_t i, int32_t j, int dir, int32_t& ni, int32_t& nj) {
    ni = i + DX[dir];
    nj = j + DZ[dir];
}

RiverNode riverNode(const TerrainParams& p, int32_t i, int32_t j) {
    const int d = down(p, i, j);
    const Info& n = info(p, i, j);
    return {n.px, n.pz, n.pot, n.sea, d, n.sea ? 0 : area(p, i, j), level(p, i, j)};
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
            const Info& n = info(p, i, j);
            if (n.sea) continue;
            const int32_t a = area(p, i, j);
            if (a < AREA_MIN) continue;
            const int64_t s = int64_t(isqrt(uint64_t(a) << 16));   // √A · 256
            const int d = down(p, i, j);
            Seg g;
            g.ax = n.px;
            g.az = n.pz;
            g.la = level(p, i, j);
            if (d >= 0) {   // река к соседу ниже
                const Info& t = info(p, i + DX[d], j + DZ[d]);
                g.bx = t.px;
                g.bz = t.pz;
                g.lb = t.sea ? p.seaLevel - 1 : (down(p, i + DX[d], j + DZ[d]) < 0 ? g.la : level(p, i + DX[d], j + DZ[d]));
                g.half = int32_t(24 + s * 14 / 256);
                g.depth = int32_t(16 + s * 6 / 256);
                g.valley = g.half + int32_t(384 + s * 80 / 256);
            } else {        // котловина - озеро
                g.bx = g.ax;
                g.bz = g.az;
                g.lb = g.la;
                g.half = int32_t(256 + s * 64 / 256);
                g.depth = int32_t(32 + s * 5 / 256);
                g.valley = g.half + 640;
            }
            // задевает ли прямоугольник (с долиной и извилистостью)
            const int64_t r = g.valley + VALLEY_CAP + WARP;
            if (std::max(g.ax, g.bx) + r < x0 || std::min(g.ax, g.bx) - r > x1 || std::max(g.az, g.bz) + r < z0 ||
                std::min(g.az, g.bz) - r > z1)
                continue;
            // земля вдоль линии (исходный рельеф без ряби мельче 64 блоков)
            const auto groundAt = [&](int64_t x, int64_t z) {
                return baseReliefAt(p, int32_t(x), int32_t(z), slowAt(p, int32_t(x), int32_t(z), GROUND_CUT), GROUND_CUT).height;
            };
            const int64_t ex = g.bx - g.ax, ez = g.bz - g.az;
            g.len = int64_t(isqrt(uint64_t(ex * ex + ez * ez)));
            if (g.len > 0) {
                // нужна только часть звена напротив прямоугольника (с долиной и извилистостью)
                int64_t umin = g.len, umax = 0;
                for (int c = 0; c < 4; ++c) {
                    const int64_t cx = ((c & 1) ? x1 + r : x0 - r) - g.ax, cz = ((c & 2) ? z1 + r : z0 - r) - g.az;
                    const int64_t u = (cx * ex + cz * ez) / g.len;
                    umin = std::min(umin, u);
                    umax = std::max(umax, u);
                }
                const int32_t last = int32_t((g.len + GROUND_STEP - 1) / GROUND_STEP);
                g.k0 = int32_t(std::clamp<int64_t>(umin / GROUND_STEP - 1, 0, last));
                const int32_t k1 = int32_t(std::clamp<int64_t>(umax / GROUND_STEP + 2, g.k0, last));
                for (int32_t k = g.k0; k <= k1; ++k) {
                    const int64_t u = std::min<int64_t>(int64_t(k) * GROUND_STEP, g.len);
                    g.ground.push_back(groundAt(g.ax + ex * u / g.len, g.az + ez * u / g.len));
                }
            } else {
                // озеро: уровень не выше самой низкой точки кольца берега минус блок - вода не переливается стеной
                int32_t rim = INT32_MAX;
                for (int k = 0; k < 8; ++k) {
                    static constexpr int RX[8] = {2, 1, 0, -1, -2, -1, 0, 1}, RZ[8] = {0, 1, 2, 1, 0, -1, -2, -1};
                    rim = std::min(rim, groundAt(g.ax + RX[k] * g.half / 2, g.az + RZ[k] * g.half / 2));
                }
                g.la = g.lb = std::max(std::min(g.la, rim - VOXELS_PER_BLOCK), p.seaLevel - 1);
            }
            segs.push_back(std::move(g));
        }
}

void RiverSet::apply(const TerrainParams& p, int32_t x, int32_t z, int32_t& h, int32_t& water, bool& shore) const {
    if (segs.empty()) return;
    // извилистость: точку запроса сдвигаем медленным шумом - русла петляют (сам рельеф этим не искажается)
    const int64_t qx = x + ((int64_t(gen::simplex2(x, z, 14, p.seed ^ 0x3A11Du)) * WARP) >> fx::SHIFT);
    const int64_t qz = z + ((int64_t(gen::simplex2(x, z, 14, p.seed ^ 0x7B23Fu)) * WARP) >> fx::SHIFT);
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
        const int64_t reach = g.valley + VALLEY_CAP;   // дальше любой долины этого звена
        if (d2 >= reach * reach) continue;
        const int32_t d = int32_t(isqrt(uint64_t(d2)));
        int32_t lw = g.la + int32_t((int64_t(g.lb - g.la) * t) >> 16);   // уровень воды здесь
        if (g.len > 0 && !g.ground.empty()) {
            // не выше земли на линии реки минус блок (точки - через GROUND_STEP от начала звена, между - линейно)
            const int64_t u = (g.len * t) >> 16;
            const int64_t k = u / GROUND_STEP, f = u - k * GROUND_STEP;
            const int64_t i0 = std::clamp<int64_t>(k - g.k0, 0, int64_t(g.ground.size()) - 1);
            const int64_t i1 = std::min<int64_t>(i0 + 1, int64_t(g.ground.size()) - 1);
            const int32_t line = g.ground[i0] + int32_t(int64_t(g.ground[i1] - g.ground[i0]) * f / GROUND_STEP);
            lw = std::max(std::min(lw, line - VOXELS_PER_BLOCK), p.seaLevel - 1);
        }
        // профиль долины: в русле - дно чашей; пойма шириной в русло - на блок выше воды; дальше стенки 1:4
        int32_t profile;
        if (d < g.half) profile = lw - g.depth + int32_t(int64_t(g.depth) * d * d / (int64_t(g.half) * g.half));
        else if (d < 2 * g.half) profile = lw + 1;
        else profile = lw + 1 + (d - 2 * g.half) / 4;
        // маска: только прорезает, к краю долины - исходный рельеф. Ширина долины - по глубине прорези в этой
        // точке: на равнине - широкая пологая, в горах - V-образная (стенки не круче ~0.75), не шире VALLEY_CAP
        const int32_t radius = std::clamp(2 * g.half + 4 * std::max(0, h0 - lw), g.valley, g.valley + VALLEY_CAP);
        const fx::q16 w = fx::ONE - fx::smoothstep(radius * 6 / 10, radius, d);
        h = std::min(h, h0 + int32_t((int64_t(w) * std::min(0, profile - h0)) >> fx::SHIFT));
        // вал у берега - насыпью: у кромки на воксель выше воды, наружу снижается 1:1, пока не встретит землю
        // (в море у устья вал не нужен - там уже вода не ниже реки)
        if (d >= g.half && water0 < lw) levee = std::max(levee, lw + 1 - (d - g.half));
        if (d < g.half) { water = std::max(water, lw); channel = true; }
        if (d < g.half + 2 * VOXELS_PER_BLOCK) shore = true;
    }
    if (!channel && levee > h) h = levee;
}

}  // namespace world
