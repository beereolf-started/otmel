#include "world/rivers.h"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <unordered_map>

#include "gen/hash.h"
#include "gen/noise.h"

namespace world {
namespace {

constexpr int64_t CELL = int64_t(1) << RIVER_CELL_LOG2;
constexpr int WINDOW = 8;                                   // окно водосбора 17×17 узлов
constexpr int CHAIN = 16;                                   // не дальше 16 звеньев
constexpr int32_t AREA_MIN = 1;                             // Р31: ручей - с первого узла водосбора
constexpr int32_t LAKE_AREA_MIN = 3;                        // озеро - с 3 узлов
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
    int64_t mouthDist = -1;         // Р31: расстояние по реке до устья (моря или озера), воксели; -1 - не считали
    int32_t mouthLevel = 0;         // Р31: уровень устья
};

// Узлы - в кэше потока: значения - чистые функции координат и зерна, порядок счёта на них не влияет.
// (Ссылки на элементы unordered_map при росте таблицы не портятся.)
struct Cache {
    std::unordered_map<uint64_t, Info> nodes;
    const TerrainParams* params = nullptr;
    uint32_t seed = 0;
};
thread_local Cache g_cache;

// Р31: готовые узлы (посчитано всё, включая уровень вдоль звена) - общие на все потоки: посчитанное одним
// потоком остальные берут готовым, а не считают сотни выборок рельефа заново
struct Shared {
    std::mutex mu;
    std::unordered_map<uint64_t, Info> done;
    const TerrainParams* params = nullptr;
    uint32_t seed = 0;
};
Shared g_shared;

uint64_t nodeKey(int32_t i, int32_t j) { return uint64_t(uint32_t(i)) << 32 | uint32_t(j); }

bool fromShared(const TerrainParams& p, uint64_t key, Info& out) {
    std::lock_guard lk(g_shared.mu);
    if (g_shared.params != &p || g_shared.seed != p.seed) {
        g_shared.done.clear();
        g_shared.params = &p;
        g_shared.seed = p.seed;
    }
    auto it = g_shared.done.find(key);
    if (it == g_shared.done.end()) return false;
    out = it->second;
    return true;
}

void publish(const TerrainParams& p, uint64_t key, const Info& n) {
    std::lock_guard lk(g_shared.mu);
    if (g_shared.params != &p || g_shared.seed != p.seed || g_shared.done.size() > 1000000) return;
    g_shared.done.try_emplace(key, n);
}

Info& info(const TerrainParams& p, int32_t i, int32_t j) {
    auto [it, fresh] = g_cache.nodes.try_emplace(nodeKey(i, j));
    Info& n = it->second;
    if (!fresh) return n;
    if (fromShared(p, nodeKey(i, j), n)) return n;
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

uint64_t isqrt(uint64_t v);
int32_t groundLine(const TerrainParams& p, int64_t x, int64_t z);

// Р31: устье узла - вниз по цепочке до моря или котловины (потенциал вниз строго падает - конечно).
// Уровень устья: море - уровень моря; котловина - земля в ней минус блок (без уровней рек - иначе круг).
void mouth(const TerrainParams& p, int32_t i, int32_t j, int64_t& dist, int32_t& lvl) {
    if (Info& n = info(p, i, j); n.mouthDist >= 0) { dist = n.mouthDist; lvl = n.mouthLevel; return; }
    const int d = down(p, i, j);
    int64_t md = 0;
    int32_t ml;
    if (d < 0) {
        const Info& n = info(p, i, j);
        ml = n.sea ? p.seaLevel - 1 : std::max(groundLine(p, n.px, n.pz), p.seaLevel - 1);
    } else {
        int64_t dd;
        mouth(p, i + DX[d], j + DZ[d], dd, ml);
        const Info& a = info(p, i, j);
        const Info& b = info(p, i + DX[d], j + DZ[d]);
        const int64_t ex = b.px - a.px, ez = b.pz - a.pz;
        md = dd + int64_t(isqrt(uint64_t(ex * ex + ez * ez)));
    }
    Info& m = info(p, i, j);
    m.mouthDist = md;
    m.mouthLevel = ml;
    dist = md;
    lvl = ml;
}

// Р30: уровень вдоль вытекающего звена - бегущий минимум земли от уровня в узле; точки через ~RUN_STEP.
// Р31: и обязательный уклон: вода падает хотя бы на 1/(64 + 24·√A) - ручьи круче, большая река положе
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
        const int64_t len = int64_t(isqrt(uint64_t(ex * ex + ez * ez)));
        const int64_t s = int64_t(isqrt(uint64_t(std::max(1, area(p, i, j))) << 16));   // √A · 256
        // уклон - по оставшейся высоте до устья: вода приходит к морю ровно к уровню моря, а не «прилипает»
        // к нему за десятки километров (широкое плоское устье); не положе 1/(256 + 64·√A)
        int64_t mdist;
        int32_t mlevel;
        mouth(p, i, j, mdist, mlevel);
        const int64_t gradeDiv = 256 + 64 * s / 256;
        int32_t cur = start;
        run.push_back(cur);
        for (int k = 1; k <= n; ++k) {
            cur = std::min(cur, groundLine(p, a.px + ex * k / n, a.pz + ez * k / n));
            const int64_t u = len * k / n;
            const int64_t byMouth = mdist > 0 ? int64_t(std::max(0, start - mlevel)) * u / mdist : 0;
            cur = std::min(cur, start - int32_t(std::max(byMouth, u / gradeDiv)));
            run.push_back(std::max(cur, p.seaLevel - 1));
        }
    }
    area(p, i, j);   // узел - целиком: публикуем вместе с полноводностью
    Info& m = info(p, i, j);
    m.run = std::move(run);
    publish(p, nodeKey(i, j), m);
    return m.run;
}

// Целый корень, точный: приближение аппаратным корнем (по IEEE 754 он округлён правильно на любой машине) и
// целочисленная поправка - результат - точное floor(√v) до бита, а не плавающее число (Р16)
uint64_t isqrt(uint64_t v) {
    uint64_t r = uint64_t(std::sqrt(double(v)));
    while (r > 0 && r * r > v) --r;
    while ((r + 1) * (r + 1) <= v) ++r;
    return r;
}

// шум вдоль реки в [-1, 1] (у каждого свойства - своё зерно)
inline fx::q16 vary(const TerrainParams& p, int32_t x, int32_t z, int cellLog2, uint32_t salt) {
    return gen::simplex2(x, z, cellLog2, p.seed ^ salt);
}

constexpr int NSTEP_LOG2 = 3;   // узлы шумов рек - через 8 вокселей

RiverSet::NoiseNode noiseNode(const TerrainParams& p, int32_t nx, int32_t nz) {
    const int32_t x = nx << NSTEP_LOG2, z = nz << NSTEP_LOG2;
    RiverSet::NoiseNode n;
    n.v[RiverSet::N_WARP] = vary(p, x, z, 15, 0x5B7u);
    n.v[RiverSet::N_WX] = vary(p, x, z, 14, 0x3A11Du);
    n.v[RiverSet::N_WZ] = vary(p, x, z, 14, 0x7B23Fu);
    n.v[RiverSet::N_WIDTH] = vary(p, x, z, 12, 0x1D7u);
    n.v[RiverSet::N_DEPTH] = vary(p, x, z, 12, 0x2E9u);
    n.v[RiverSet::N_PLAIN] = vary(p, x, z, 13, 0x3F1u);
    n.v[RiverSet::N_BANK] = vary(p, x, z, 13, 0x4A3u);
    n.v[RiverSet::N_EDGE] = vary(p, x, z, 9, 0x6C1u);
    n.v[RiverSet::N_BANKL] = vary(p, x, z, 9, 0x7D3u);
    n.v[RiverSet::N_BED] = vary(p, x, z, 8, 0x8E5u);
    n.v[RiverSet::N_B1] = vary(p, x, z, 8, 0x9F7u);
    n.v[RiverSet::N_B2] = vary(p, x, z, 7, 0xA19u);
    n.v[RiverSet::N_BRAID] = vary(p, x, z, 11, 0xB2Bu);
    n.v[RiverSet::N_WX2] = vary(p, x, z, 13, 0xC3Du);
    n.v[RiverSet::N_WZ2] = vary(p, x, z, 13, 0xD4Fu);
    return n;
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
            const int d = down(p, i, j);
            // Р31: ручьи - с первого узла водосбора (видно всё дерево притоков); озёра - с 3 узлов
            if (a < (d >= 0 ? AREA_MIN : LAKE_AREA_MIN)) continue;
            const int64_t s = int64_t(isqrt(uint64_t(a) << 16));   // √A · 256
            Seg g;
            g.area = a;
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
            // шаг плёс-перекат - около 12 ширин русла, в пределах 24..160 блоков
            // шаг плёс-перекат - около 24 ширин русла, 48..240 блоков: падение собирается в заметные перекаты
            g.pool = std::clamp(24 * 2 * g.half, 48 * VOXELS_PER_BLOCK, 240 * VOXELS_PER_BLOCK);
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
            g.minLevel = *std::min_element(g.run.begin(), g.run.end());
            // шаг плёс-перекат и по уклону: на перекат - не больше ~4 блоков падения (в горах - лестница каскадов,
            // на равнине - длинные плёсы); настоящий обрыв земли всё равно даёт большой водопад
            if (g.len > 0 && g.run.front() > g.run.back()) {
                const int64_t byDrop = int64_t(4 * VOXELS_PER_BLOCK) * g.len / (g.run.front() - g.run.back());
                g.pool = int32_t(std::clamp<int64_t>(std::min<int64_t>(g.pool, byDrop), 8 * VOXELS_PER_BLOCK, g.pool));
            }
            segs.push_back(std::move(g));
        }
    // Р31: шумы рек - узлы сетки над прямоугольником (если он мал - кусок; у плиток вершины и так на узлах)
    grid.clear();
    gw = gh = 0;
    if (!segs.empty() && int64_t(x1) - x0 <= 512 && int64_t(z1) - z0 <= 512) {
        gnx0 = x0 >> NSTEP_LOG2;
        gnz0 = z0 >> NSTEP_LOG2;
        gw = (x1 >> NSTEP_LOG2) - gnx0 + 2;
        gh = (z1 >> NSTEP_LOG2) - gnz0 + 2;
        grid.resize(size_t(gw) * gh);
        for (int j = 0; j < gh; ++j)
            for (int i = 0; i < gw; ++i) grid[size_t(j) * gw + i] = noiseNode(p, gnx0 + i, gnz0 + j);
    }
}

void RiverSet::noiseAt(const TerrainParams& p, int32_t x, int32_t z, fx::q16 out[N_COUNT]) const {
    constexpr int32_t mask = (1 << NSTEP_LOG2) - 1;
    const int32_t nx = x >> NSTEP_LOG2, nz = z >> NSTEP_LOG2;
    const fx::q16 fxr = (x & mask) << (fx::SHIFT - NSTEP_LOG2), fz = (z & mask) << (fx::SHIFT - NSTEP_LOG2);
    const auto node = [&](int32_t i, int32_t j) {
        const int32_t gi = i - gnx0, gj = j - gnz0;
        if (gi >= 0 && gj >= 0 && gi < gw && gj < gh) return grid[size_t(gj) * gw + gi];
        return noiseNode(p, i, j);
    };
    // узел с нулевым весом не спрашиваем (точка на линии сетки) - lerp(a, b, 0) = a точно
    NoiseNode top = node(nx, nz);
    if (fxr) { const NoiseNode r = node(nx + 1, nz); for (int k = 0; k < N_COUNT; ++k) top.v[k] = fx::lerp(top.v[k], r.v[k], fxr); }
    if (fz) {
        NoiseNode bot = node(nx, nz + 1);
        if (fxr) { const NoiseNode r = node(nx + 1, nz + 1); for (int k = 0; k < N_COUNT; ++k) bot.v[k] = fx::lerp(bot.v[k], r.v[k], fxr); }
        for (int k = 0; k < N_COUNT; ++k) top.v[k] = fx::lerp(top.v[k], bot.v[k], fz);
    }
    for (int k = 0; k < N_COUNT; ++k) out[k] = top.v[k];
}

// уровень бегущего минимума в точке u звена (точки run - равномерно по длине, между ними - линейно)
static int32_t runAt(const RiverSet::Seg& g, int64_t u) {
    if (g.run.size() == 1 || g.len <= 0) return g.run[0];
    const int64_t n = int64_t(g.run.size()) - 1;
    u = std::clamp<int64_t>(u, 0, g.len);
    const int64_t num = u * n;
    const int64_t k = std::min<int64_t>(num / g.len, n - 1);
    const int64_t f = num - k * g.len;   // доля шага · len
    return g.run[size_t(k)] + int32_t(int64_t(g.run[size_t(k) + 1] - g.run[size_t(k)]) * f / g.len);
}

int32_t RiverSet::levelAlong(const Seg& g, int64_t u, int32_t* drop) {
    if (drop) *drop = 0;
    if (g.run.size() == 1 || g.len <= 0) return g.run[0];
    // Р31: плёсы и перекаты. Плёс k - [k·S, (k+1)·S): ровный, на уровне своего нижнего конца (поэтому вода не
    // выше земли - уровень бегущего минимума вниз только падает); последняя четверть шага - перекат вниз к
    // уровню следующего плёса. Всё падение участка собрано в перекат: в горах он - порог или водопад.
    const int64_t S = g.pool, r = std::max<int64_t>(S / 4, 1);
    u = std::clamp<int64_t>(u, 0, g.len);
    const int64_t k = u / S;
    const int64_t endK = std::min((k + 1) * S, g.len), endNext = std::min((k + 2) * S, g.len);
    const int32_t pool = runAt(g, endK), next = runAt(g, endNext);
    if (u <= endK - r || endK >= g.len) return pool;
    if (drop) *drop = pool - next;
    return pool + int32_t(int64_t(next - pool) * (u - (endK - r)) / r);
}

void RiverSet::apply(const TerrainParams& p, int32_t x, int32_t z, int32_t& h, int32_t& water, bool& shore) const {
    if (segs.empty()) return;
    fx::q16 nv[N_COUNT];
    noiseAt(p, x, z, nv);   // Р31: шумы рек - из сетки узлов
    // извилистость: точку запроса сдвигаем медленным шумом - русла петляют (сам рельеф этим не искажается)
    const fx::q16 kWarp = fx::ONE * 9 / 10 + nv[N_WARP] * 6 / 10;      // извилистость ×0.3..1.5
    const int64_t warp = (int64_t(WARP) * kWarp) >> fx::SHIFT;
    const int64_t qx = x + ((int64_t(nv[N_WX]) * warp) >> fx::SHIFT);
    const int64_t qz = z + ((int64_t(nv[N_WZ]) * warp) >> fx::SHIFT);
    const int32_t h0 = h, water0 = water;   // water0 - вода до рек (море)
    // расстояние до звена от сдвинутой точки (q) и доля пути t (Q16)
    const auto nearest = [](const Seg& g, int64_t qx_, int64_t qz_, int64_t& t) {
        const int64_t ex = g.bx - g.ax, ez = g.bz - g.az, px = qx_ - g.ax, pz = qz_ - g.az;
        const int64_t len2 = ex * ex + ez * ez;
        t = 0;
        if (len2 > 0) t = std::clamp<int64_t>(((px * ex + pz * ez) << 16) / len2, 0, fx::ONE);
        const int64_t dx = px - ((ex * t) >> 16), dz = pz - ((ez * t) >> 16);
        return dx * dx + dz * dz;
    };

    // Проход 1 (дёшево): какие звенья вообще могут задеть этот столбец - по грубой верхней границе долины
    // (наибольшие ширина русла, пойма и заложение стенок). Шумы считаем, только если такие есть.
    struct Near { const Seg* g; int32_t d, lw; int64_t t; };
    Near nearBuf[32];
    int nNear = 0;
    bool braidable = false;
    for (const Seg& g : segs) {
        int64_t t;
        const int64_t d2 = nearest(g, qx, qz, t);
        const int64_t reach = 2 * g.valley + VALLEY_CAP;
        if (d2 >= reach * reach) continue;
        // грубая отсечка ДО уровня: уровень здесь не ниже нижнего уровня звена
        const int64_t rup = std::clamp<int64_t>(8 * int64_t(g.half) + 11 * std::max(0, h0 - g.minLevel), g.valley,
                                                g.valley + VALLEY_CAP) + 8 * g.half;
        if (d2 >= rup * rup) continue;
        const int32_t lw = levelAlong(g, (g.len * t) >> 16);
        const int64_t rmax = std::clamp<int64_t>(8 * int64_t(g.half) + 11 * std::max(0, h0 - lw), g.valley, g.valley + VALLEY_CAP);
        if (d2 >= (rmax + 8 * g.half) * (rmax + 8 * g.half)) continue;
        const int32_t d = int32_t(isqrt(uint64_t(d2)));
        if (nNear < 32) nearBuf[nNear++] = {&g, d, lw, t};
        braidable |= g.area >= 20 && g.len > 0;
    }
    if (nNear == 0) return;

    // Проход 2: шумы. Р30: медленные (ширина, глубина, пойма, стенки); Р31: местные - неровная кромка (32 блока),
    // бугры поймы (16 и 8 блоков), неровные берега и дно
    const fx::q16 nWidth = nv[N_WIDTH], nDepth = nv[N_DEPTH], nPlain = nv[N_PLAIN], nBank = nv[N_BANK];
    const fx::q16 nEdge = nv[N_EDGE], nBankL = nv[N_BANKL], nBed = nv[N_BED], b1 = nv[N_B1], b2 = nv[N_B2];
    const int32_t bump = int32_t(((int64_t(std::abs(b1)) * 2 + std::abs(b2)) * (2 * VOXELS_PER_BLOCK)) / (3 * int64_t(fx::ONE)));
    const fx::q16 kWidth = fx::mul(fx::ONE * 11 / 10 + nWidth / 2, fx::ONE + nEdge / 4);   // ×0.6..1.6, кромка ±25%
    const fx::q16 kDepth = fx::mul(fx::ONE + nDepth * 3 / 10, fx::ONE + nBed / 5);         // ±30%, дно ±20%
    const fx::q16 kPlain = fx::ONE * 7 / 4 + nPlain * 5 / 4;                               // пойма ×0.5..3
    // стенки долины 1:2..1:8, местами ±30% - заложение bankRun (Q16, непрерывно: целый делитель прыгал бы)
    const int64_t bankRun = (int64_t(5 * fx::ONE + 3 * int64_t(nBank)) * (fx::ONE + nBankL * 3 / 10)) >> fx::SHIFT;
    // Р31: рукава - второе русло с другой извилистостью, у больших рек на ровных участках
    fx::q16 braid = 0;
    int64_t qx2 = 0, qz2 = 0;
    if (braidable) {
        braid = fx::smoothstep(fx::ONE * 15 / 100, fx::ONE * 35 / 100, nv[N_BRAID]);
        qx2 = x + ((int64_t(nv[N_WX2]) * warp) >> fx::SHIFT);
        qz2 = z + ((int64_t(nv[N_WZ2]) * warp) >> fx::SHIFT);
    }

    int32_t levee = INT32_MIN;   // вал: земля не ниже воды у берега (если точка не в русле и не в море)
    bool channel = false;
    for (int k = 0; k < nNear; ++k) {
        const Seg& g = *nearBuf[k].g;
        const int32_t d = nearBuf[k].d, lw = nearBuf[k].lw;
        const int32_t half = std::max<int32_t>(16, int32_t((int64_t(g.half) * (g.len > 0 ? kWidth : fx::ONE)) >> fx::SHIFT));
        const int32_t depth = int32_t((int64_t(g.depth) * kDepth) >> fx::SHIFT);
        // у моря пойма сужается (уровень воды в 8 блоках от моря - до нуля): устье - русло с берегами, а не разлив
        const fx::q16 nearSea = fx::smoothstep(p.seaLevel, p.seaLevel + 8 * VOXELS_PER_BLOCK, lw);
        const int32_t plain = half + int32_t((int64_t(half) * fx::mul(kPlain, nearSea)) >> fx::SHIFT);
        const int32_t radius = int32_t(std::clamp<int64_t>(plain + ((bankRun * std::max(0, h0 - lw)) >> fx::SHIFT),
                                                           g.valley, g.valley + VALLEY_CAP));
        // за краем долины, берега и вала (и рукава здесь быть не может) - звено столбец не меняет
        if (d >= radius && d >= half + 4 * VOXELS_PER_BLOCK && (water0 != NO_WATER || d >= half + (lw + 1 - h0)) &&
            !(braid > 0 && g.area >= 20 && g.len > 0))
            continue;
        // профиль: в русле - дно чашей; пойма - на блок выше воды, бугры нарастают от кромки за 3 блока;
        // дальше стенки с заложением bankRun. Непрерывен (на кромке - ступенька в воксель).
        const auto dish = [&](int32_t dd, int32_t hh) { return lw - depth + int32_t(int64_t(depth) * dd * dd / (int64_t(hh) * hh)); };
        const int32_t bumpHere = int32_t((int64_t(bump) * fx::smoothstep(half, half + 3 * VOXELS_PER_BLOCK, d)) >> fx::SHIFT);
        int32_t profile;
        if (d < half) profile = dish(d, half);
        else if (d < plain) profile = lw + 1 + bumpHere;
        else profile = lw + 1 + bumpHere + int32_t((int64_t(d - plain) << fx::SHIFT) / bankRun);
        // рукав: у рек от 20 узлов, только на ровном (вода у земли) - второе русло той же глубины и уровня,
        // вокруг него откос 1:1 (через минимум с основным профилем - без ступенек)
        bool inBraid = false;
        if (braid > 0 && g.area >= 20 && g.len > 0) {
            const fx::q16 flat = fx::ONE - fx::smoothstep(0, 4 * VOXELS_PER_BLOCK, h0 - lw);
            const int32_t half2 = int32_t((int64_t(half) * 6 / 10 * fx::mul(braid, flat)) >> fx::SHIFT);
            if (half2 > 8) {
                int64_t t2;
                const int64_t e2 = nearest(g, qx2, qz2, t2);
                const int64_t outer = int64_t(half2) + 4 * VOXELS_PER_BLOCK;
                if (e2 < outer * outer) {
                    const int32_t d2b = int32_t(isqrt(uint64_t(e2)));
                    profile = std::min(profile, d2b < half2 ? dish(d2b, half2) : lw + (d2b - half2));
                    inBraid = d2b < half2;
                }
            }
        }
        // маска: только прорезает, к краю долины - исходный рельеф. Ширина долины - по глубине прорези в этой
        // точке (radius выше): на равнине - широкая пологая, в горах - V-образная, не шире VALLEY_CAP
        const fx::q16 w = fx::ONE - fx::smoothstep(radius * 6 / 10, radius, d);
        h = std::min(h, h0 + int32_t((int64_t(w) * std::min(0, profile - h0)) >> fx::SHIFT));
        // вал у берега - насыпью: у кромки на воксель выше воды, наружу снижается 1:1, пока не встретит землю
        // (вал - только на сухой земле: в море у устья он давал «волнорез» вдоль продолжения русла)
        if (d >= half && !inBraid && water0 == NO_WATER) levee = std::max(levee, lw + 1 - (d - half));
        if (d < half || inBraid) { water = std::max(water, lw); channel = true; }
        if (d < half + 2 * VOXELS_PER_BLOCK) shore = true;
    }
    if (!channel && levee > h) h = levee;
}

RiverSet::Current RiverSet::current(const TerrainParams& p, int32_t x, int32_t z) const {
    // Р31: течение в русле - вдоль звена (от истока к устью); скорость - как в гидравлике (Маннинг):
    // растёт с глубиной (полноводность) и с падением уровня (крутизна): плёс - медленно, перекат - быстро,
    // водопад - предел. v = 8 · √(уклон) · √(глубина в блоках) блоков/с, от 0.2 до 12.
    Current c{};
    fx::q16 nv[N_COUNT];
    noiseAt(p, x, z, nv);
    const int64_t warp = (int64_t(WARP) * (fx::ONE * 9 / 10 + nv[N_WARP] * 6 / 10)) >> fx::SHIFT;
    const int64_t qx = x + ((int64_t(nv[N_WX]) * warp) >> fx::SHIFT);
    const int64_t qz = z + ((int64_t(nv[N_WZ]) * warp) >> fx::SHIFT);
    int64_t best = INT64_MAX;
    for (const Seg& g : segs) {
        if (g.len <= 0) continue;
        const int64_t ex = g.bx - g.ax, ez = g.bz - g.az, px = qx - g.ax, pz = qz - g.az;
        const int64_t t = std::clamp<int64_t>(((px * ex + pz * ez) << 16) / (ex * ex + ez * ez), 0, fx::ONE);
        const int64_t dx = px - ((ex * t) >> 16), dz = pz - ((ez * t) >> 16);
        const int64_t d2 = dx * dx + dz * dz;
        if (d2 >= int64_t(g.half) * g.half * 3 || d2 >= best) continue;   // в русле (с запасом на разброс ширины)
        best = d2;
        const int64_t u = (g.len * t) >> 16, step = 16;
        const int32_t l0 = levelAlong(g, u), l1 = levelAlong(g, u + step);
        const int64_t slope = (int64_t(std::max(0, l0 - l1)) << fx::SHIFT) / step;   // Q16, вокселей на воксель
        const int64_t depthBlocks = std::max<int64_t>(1, g.depth / VOXELS_PER_BLOCK);
        int64_t v = 8 * int64_t(isqrt(uint64_t(slope) << 16)) * int64_t(isqrt(uint64_t(depthBlocks) << 32)) >> 16;   // Q16
        v = std::clamp<int64_t>(v, fx::ONE / 5, 12 * int64_t(fx::ONE));
        c.dirX = fx::q16((ex << fx::SHIFT) / g.len);
        c.dirZ = fx::q16((ez << fx::SHIFT) / g.len);
        c.speed = fx::q16(v);
        c.inRiver = true;
    }
    return c;
}

}  // namespace world
