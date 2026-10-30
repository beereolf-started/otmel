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
constexpr int WINDOW = 16;                                  // Р36: окно водосбора 33×33 узла (~66 км)
constexpr int CHAIN = 32;                                   // не дальше 32 звеньев
constexpr int32_t AREA_MIN = 1;                             // Р31: ручей - с первого узла водосбора
constexpr int32_t LAKE_AREA_MIN = 3;                        // озеро - с 3 узлов
constexpr int32_t VALLEY_CAP = 400 * VOXELS_PER_BLOCK;      // долина по глубине прорези - не шире на столько
constexpr int SMOOTH_CUT = 15;                              // сглаженная высота - без форм мельче 2048 блоков
// уклон к морю: потенциал стока = сглаженная высота + материковость · TILT; материковость падает к океану на
// ~10 км - мелкие впадины не ловят сток, реки тянутся к побережью, хребты (сотни блоков) по-прежнему главнее
constexpr int64_t TILT = 200 * VOXELS_PER_BLOCK;
constexpr int GROUND_CUT = 9;                               // земля вдоль реки - без ряби мельче 32 блоков
constexpr int DX[8] = {1, 1, 0, -1, -1, -1, 0, 1}, DZ[8] = {0, 1, 1, 1, 0, -1, -1, -1};
// Р32: русло - поиск ложбины. Станции через 64 блока, на каждой 13 вариантов сдвига вбок в коридоре ±400 блоков;
// путь с наименьшей суммой высот земли плюс штраф за каждый шаг вбок (не больше шага на станцию - не круче 46°)
constexpr int64_t STATION = 64 * VOXELS_PER_BLOCK;
constexpr int SIDE = 6;
constexpr int64_t SPREAD = 400 * VOXELS_PER_BLOCK;
constexpr int64_t BEND = 4 * VOXELS_PER_BLOCK;
constexpr int64_t MEANDER = 100 * VOXELS_PER_BLOCK;         // меандры поверх найденного пути: до ±100 блоков ×0.3..1.5
constexpr int64_t ANCHOR = 192 * VOXELS_PER_BLOCK;          // узел садится в самую низкую из 3×3 точек через 192 блока
constexpr int32_t SPILL = 2 * VOXELS_PER_BLOCK;             // русло над чужой долиной выше этого - вода сливается
constexpr int SPILL_R = 5;                                  // Р37: выход котловины ищется не дальше 5 узлов (~10 км)

struct Info {
    int64_t px, pz;
    int32_t smooth;     // сглаженная высота
    int32_t pot;        // потенциал стока: сглаженная высота + уклон к морю
    bool sea;
    int down = -2;      // -2: ещё не считали; 0..7 - сосед; 8 - сток котловины через перевал (Р37)
    int32_t ti = 0, tj = 0;   // куда стекает
    int32_t area = -1;  // -1: ещё не считали
    bool anchored = false;          // Р32: точка русла в узле (самая низкая около узла)
    int64_t ax = 0, az = 0;
    bool leveled = false;
    int32_t level = 0;              // Р30: уровень воды в узле
    std::shared_ptr<const RiverPath> path;   // Р32: вытекающее звено (пусто - ещё не считали)
    bool trimmed = false;                   // Р33: русло для рельефа - обрезано при впадении в реку ниже
    std::shared_ptr<const RiverPath> drawPath;
    int64_t mouthDist = -1;         // Р31: расстояние до устья (моря или озера), воксели; -1 - не считали
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

// Р31: готовые узлы (посчитано всё, включая русло) - общие на все потоки: посчитанное одним
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

// Р37: сток котловины. Узел, у которого все соседи выше (котловина), стекает через самый низкий перевал: волна от
// котловины по узлам в радиусе SPILL_R, в порядке высоты прохода (наибольший потенциал на пути); первый узел НИЖЕ
// котловины по потенциалу - выход. Связь - прямо к нему (русло проложит поиск ложбины, через перевал - ущельем).
// Каждая связь ведёт строго вниз по потенциалу - кругов нет. Выхода в радиусе нет - бессточное озеро.
bool spillOutlet(const TerrainParams& p, int32_t i, int32_t j, int32_t& ti, int32_t& tj) {
    constexpr int W = 2 * SPILL_R + 1;
    const int32_t pot0 = info(p, i, j).pot;
    int32_t key[W][W];
    bool done[W][W] = {};
    for (auto& row : key) for (int32_t& k : row) k = INT32_MAX;
    key[SPILL_R][SPILL_R] = pot0;
    for (;;) {
        int bi = -1, bj = -1;
        int32_t bk = INT32_MAX;
        for (int b = 0; b < W; ++b)
            for (int a = 0; a < W; ++a)
                if (!done[b][a] && key[b][a] < bk) { bk = key[b][a]; bi = a; bj = b; }
        if (bi < 0) return false;
        done[bj][bi] = true;
        const int32_t ci = i + bi - SPILL_R, cj = j + bj - SPILL_R;
        const Info& c = info(p, ci, cj);
        if ((bi != SPILL_R || bj != SPILL_R) && c.pot < pot0) { ti = ci; tj = cj; return true; }
        for (int d = 0; d < 8; ++d) {
            const int a = bi + DX[d], b = bj + DZ[d];
            if (a < 0 || b < 0 || a >= W || b >= W || done[b][a]) continue;
            const int32_t k = std::max(bk, info(p, i + a - SPILL_R, j + b - SPILL_R).pot);
            if (k < key[b][a]) key[b][a] = k;
        }
    }
}

int down(const TerrainParams& p, int32_t i, int32_t j) {
    Info& n = info(p, i, j);
    if (n.down != -2) return n.down;
    int best = -1;
    int32_t lowest = n.pot, ti = 0, tj = 0;
    if (!n.sea) {
        for (int d = 0; d < 8; ++d) {
            const int32_t s = info(p, i + DX[d], j + DZ[d]).pot;
            if (s < lowest) { lowest = s; best = d; }   // строго ниже по потенциалу - иначе котловина
        }
        if (best >= 0) { ti = i + DX[best]; tj = j + DZ[best]; }
        else if (spillOutlet(p, i, j, ti, tj)) best = 8;
    }
    Info& m = info(p, i, j);
    m.down = best;
    m.ti = ti;
    m.tj = tj;
    return best;
}

// куда стекает узел (false - никуда)
bool target(const TerrainParams& p, int32_t i, int32_t j, int32_t& ti, int32_t& tj) {
    if (down(p, i, j) < 0) return false;
    const Info& n = info(p, i, j);
    ti = n.ti;
    tj = n.tj;
    return true;
}

int32_t area(const TerrainParams& p, int32_t i, int32_t j) {
    if (int32_t a = info(p, i, j).area; a >= 0) return a;
    int32_t count = 0;
    for (int dj = -WINDOW; dj <= WINDOW; ++dj)
        for (int di = -WINDOW; di <= WINDOW; ++di) {
            int32_t ci = i + di, cj = j + dj;
            for (int step = 0; step <= CHAIN; ++step) {
                if (ci == i && cj == j) { ++count; break; }
                int32_t ni, nj;
                if (!target(p, ci, cj, ni, nj)) break;
                ci = ni;
                cj = nj;
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

// Целый корень, точный: приближение аппаратным корнем (по IEEE 754 он округлён правильно на любой машине) и
// целочисленная поправка - результат - точное floor(√v) до бита, а не плавающее число (Р16)
uint64_t isqrt(uint64_t v) {
    uint64_t r = uint64_t(std::sqrt(double(v)));
    while (r > 0 && r * r > v) --r;
    while ((r + 1) * (r + 1) <= v) ++r;
    return r;
}

// Р32: точка русла в узле - самая низкая земля из 3×3 точек через ANCHOR: узел, сдвинутый хешем на холм, не
// тянет на этот холм все свои реки
void anchor(const TerrainParams& p, int32_t i, int32_t j, int64_t& ax, int64_t& az) {
    if (Info& n = info(p, i, j); n.anchored) { ax = n.ax; az = n.az; return; }
    const Info& n = info(p, i, j);
    int64_t bx = n.px, bz = n.pz;
    int32_t best = groundLine(p, bx, bz);
    for (int dz = -1; dz <= 1; ++dz)
        for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dz == 0) continue;
            const int64_t x = n.px + dx * ANCHOR, z = n.pz + dz * ANCHOR;
            const int32_t g = groundLine(p, x, z);
            if (g < best) { best = g; bx = x; bz = z; }
        }
    Info& m = info(p, i, j);
    m.ax = ax = bx;
    m.az = az = bz;
    m.anchored = true;
}

const RiverPath& outPath(const TerrainParams& p, int32_t i, int32_t j);

// Р30: уровень воды в узле - минимум земли в нём и концов всех впадающих звеньев (притоков); по дереву стока
// вверх до истоков (потенциал строго растёт вверх - рекурсия конечна)
int32_t level(const TerrainParams& p, int32_t i, int32_t j) {
    if (Info& n = info(p, i, j); n.leveled) return n.level;
    int64_t ax, az;
    anchor(p, i, j, ax, az);
    int32_t lv = groundLine(p, ax, az);
    // Р38: и вокруг узла - на две ширины русла (как поперёк русла в вершинах): у истока вода стояла кубиком над
    // землёй, которая рядом с точкой узла ниже
    if (!info(p, i, j).sea) {
        const int64_t s = int64_t(isqrt(uint64_t(std::max(1, area(p, i, j))) << 16));
        const int64_t r = 2 * 2 * (24 + s * 14 / 256) + 2 * VOXELS_PER_BLOCK;
        lv = std::min({lv, groundLine(p, ax + r, az), groundLine(p, ax - r, az), groundLine(p, ax, az + r), groundLine(p, ax, az - r)});
    }
    // притоки: соседи, стекающие сюда, и котловины в радиусе SPILL_R, чей выход - здесь (Р37)
    for (int32_t cj = j - SPILL_R; cj <= j + SPILL_R; ++cj)
        for (int32_t ci = i - SPILL_R; ci <= i + SPILL_R; ++ci) {
            int32_t ti, tj;
            if ((ci != i || cj != j) && target(p, ci, cj, ti, tj) && ti == i && tj == j)
                lv = std::min(lv, outPath(p, ci, cj).run.back());
        }
    Info& m = info(p, i, j);
    m.level = std::max(lv, p.seaLevel - 1);
    m.leveled = true;
    return m.level;
}

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
        const int32_t ti = info(p, i, j).ti, tj = info(p, i, j).tj;
        mouth(p, ti, tj, dd, ml);
        const Info& a = info(p, i, j);
        const Info& b = info(p, ti, tj);
        const int64_t ex = b.px - a.px, ez = b.pz - a.pz;
        md = dd + int64_t(isqrt(uint64_t(ex * ex + ez * ez)));
    }
    Info& m = info(p, i, j);
    m.mouthDist = md;
    m.mouthLevel = ml;
    dist = md;
    lvl = ml;
}

// Р32: ломаная русла от a к b. 1) поиск ложбины - динамическое программирование по станциям; 2) меандры -
// сдвиг станций вбок шумом (у концов - на нет, на крутом - меньше); 3) два раза срезание углов (Чайкин) -
// вершины через ~16 блоков. Сдвиги - вдоль одной нормали к прямой a-b: ломаная - график над прямой, сама себя
// не пересекает.
void tracePath(const TerrainParams& p, int64_t ax, int64_t az, int64_t bx, int64_t bz, RiverPath& out) {
    const int64_t ex = bx - ax, ez = bz - az;
    const int64_t len = std::max<int64_t>(1, int64_t(isqrt(uint64_t(ex * ex + ez * ez))));
    const int n = int(std::max<int64_t>(2, (len + STATION / 2) / STATION));
    constexpr int K = 2 * SIDE + 1;
    constexpr int64_t INF = INT64_MAX / 4;
    // станция s, вариант k: точка на прямой + нормаль (-ez, ex)/len · сдвиг
    const auto offsetOf = [](int k) { return SPREAD * (k - SIDE) / SIDE; };
    const auto at = [&](int s, int64_t off, int64_t& x, int64_t& z) {
        x = ax + ex * s / n - ez * off / len;
        z = az + ez * s / n + ex * off / len;
    };
    std::vector<int64_t> cost(K, INF), next(K);
    std::vector<uint8_t> from(size_t(n + 1) * K, uint8_t(SIDE));
    cost[SIDE] = 0;
    for (int s = 1; s <= n; ++s) {
        for (int k = 0; k < K; ++k) {
            next[k] = INF;
            if (s == n && k != SIDE) continue;                    // конец - в узле
            if (std::abs(k - SIDE) > std::min(s, n - s)) continue;   // от узлов не дальше шага на станцию
            for (int kp = std::max(0, k - 1); kp <= std::min(K - 1, k + 1); ++kp) {
                if (cost[kp] >= INF) continue;
                const int64_t c = cost[kp] + BEND * std::abs(k - kp);
                if (c < next[k]) { next[k] = c; from[size_t(s) * K + k] = uint8_t(kp); }
            }
            if (next[k] < INF && s < n) {
                int64_t x, z;
                at(s, offsetOf(k), x, z);
                next[k] += groundLine(p, x, z);
            }
        }
        std::swap(cost, next);
    }
    std::vector<int64_t> off(n + 1, 0);
    for (int s = n, k = SIDE; s > 0; --s) {
        off[s] = offsetOf(k);
        k = from[size_t(s) * K + k];
    }
    off[0] = 0;
    // меандры: на равнине во всю силу, на крутом (уклон земли от 1/30) - почти прямо, как у горных рек
    const int64_t drop = std::max<int64_t>(0, groundLine(p, ax, az) - groundLine(p, bx, bz));
    const fx::q16 grade = fx::q16(std::min<int64_t>(fx::ONE, (drop << fx::SHIFT) / len));
    const fx::q16 flat = fx::ONE - fx::smoothstep(fx::ONE / 200, fx::ONE / 30, grade);
    const fx::q16 kAmp = fx::ONE * 9 / 10 + gen::simplex2(int32_t(ax), int32_t(az), 15, p.seed ^ 0x5B7u) * 6 / 10;
    const int64_t amp = (((MEANDER * kAmp) >> fx::SHIFT) * (fx::ONE / 5 + flat * 4 / 5)) >> fx::SHIFT;
    std::vector<int64_t> px(n + 1), pz(n + 1);
    for (int s = 0; s <= n; ++s) {
        int64_t x, z;
        at(s, 0, x, z);
        const int64_t taper = std::min<int64_t>(fx::ONE, int64_t(std::min(s, n - s)) * fx::ONE / 3);
        const int64_t m = (((int64_t(gen::simplex2(int32_t(x), int32_t(z), 13, p.seed ^ 0x3A11Du)) * amp) >> fx::SHIFT) * taper) >> fx::SHIFT;
        at(s, off[s] + m, px[s], pz[s]);
    }
    // срезание углов: каждое ребро QR -> точки 3/4Q+1/4R и 1/4Q+3/4R; концы на месте
    for (int it = 0; it < 2; ++it) {
        std::vector<int64_t> nx{px.front()}, nz{pz.front()};
        for (size_t k = 0; k + 1 < px.size(); ++k) {
            nx.push_back((3 * px[k] + px[k + 1]) / 4);
            nz.push_back((3 * pz[k] + pz[k + 1]) / 4);
            nx.push_back((px[k] + 3 * px[k + 1]) / 4);
            nz.push_back((pz[k] + 3 * pz[k + 1]) / 4);
        }
        nx.push_back(px.back());
        nz.push_back(pz.back());
        px.swap(nx);
        pz.swap(nz);
    }
    out.x = std::move(px);
    out.z = std::move(pz);
}

// вершины, длины, рамки групп рёбер
void finishPath(RiverPath& r) {
    const size_t nv = r.x.size();
    r.cum.assign(nv, 0);
    for (size_t k = 1; k < nv; ++k) {
        const int64_t dx = r.x[k] - r.x[k - 1], dz = r.z[k] - r.z[k - 1];
        r.cum[k] = r.cum[k - 1] + int64_t(isqrt(uint64_t(dx * dx + dz * dz)));
    }
    r.len = r.cum.back();
    r.boxes.clear();
    r.supers.clear();
    r.boxLow.clear();
    r.superLow.clear();
    r.bounds = {r.x[0], r.z[0], r.x[0], r.z[0]};
    const size_t ne = nv > 1 ? nv - 1 : 1;
    for (size_t g = 0; g < ne; g += RiverPath::GROUP) {
        RiverPath::Box b{INT64_MAX, INT64_MAX, INT64_MIN, INT64_MIN};
        for (size_t k = g; k <= std::min(g + RiverPath::GROUP, nv - 1); ++k) {
            b.x0 = std::min(b.x0, r.x[k]);
            b.z0 = std::min(b.z0, r.z[k]);
            b.x1 = std::max(b.x1, r.x[k]);
            b.z1 = std::max(b.z1, r.z[k]);
        }
        int32_t low = INT32_MAX;
        if (r.run.size() == nv)
            for (size_t k = g; k <= std::min(g + RiverPath::GROUP, nv - 1); ++k) low = std::min(low, r.run[k]);
        r.boxes.push_back(b);
        r.boxLow.push_back(low);
        if (r.boxes.size() % RiverPath::SUPER == 1) { r.supers.push_back(b); r.superLow.push_back(low); }
        r.superLow.back() = std::min(r.superLow.back(), low);
        RiverPath::Box& sb = r.supers.back();
        sb.x0 = std::min(sb.x0, b.x0);
        sb.z0 = std::min(sb.z0, b.z0);
        sb.x1 = std::max(sb.x1, b.x1);
        sb.z1 = std::max(sb.z1, b.z1);
        r.bounds.x0 = std::min(r.bounds.x0, b.x0);
        r.bounds.z0 = std::min(r.bounds.z0, b.z0);
        r.bounds.x1 = std::max(r.bounds.x1, b.x1);
        r.bounds.z1 = std::max(r.bounds.z1, b.z1);
    }
}

// Р30: уровень вдоль вытекающего звена - бегущий минимум земли от уровня в узле, в каждой вершине русла (Р32).
// Р31: и обязательный уклон по оставшейся высоте до устья, не положе 1/(256 + 64·√A)
const RiverPath& outPath(const TerrainParams& p, int32_t i, int32_t j) {
    if (const Info& n = info(p, i, j); n.path) return *n.path;
    const int32_t start = level(p, i, j);
    const int d = down(p, i, j);
    auto path = std::make_shared<RiverPath>();
    int64_t ax, az;
    anchor(p, i, j, ax, az);
    if (d < 0) {
        path->x = {ax};
        path->z = {az};
        path->run = {start};
    } else {
        int64_t bx, bz;
        const int32_t ti = info(p, i, j).ti, tj = info(p, i, j).tj;
        anchor(p, ti, tj, bx, bz);
        tracePath(p, ax, az, bx, bz, *path);
        const int64_t s = int64_t(isqrt(uint64_t(std::max(1, area(p, i, j))) << 16));   // √A · 256
        int64_t mdist;
        int32_t mlevel;
        mouth(p, i, j, mdist, mlevel);
        const int64_t gradeDiv = 256 + 64 * s / 256;
        // путь по руслу длиннее прямой между узлами: доля пути до устья - по доле длины русла
        const Info& a = info(p, i, j);
        const Info& b = info(p, ti, tj);
        const int64_t sx = b.px - a.px, sz = b.pz - a.pz;
        const int64_t straight = int64_t(isqrt(uint64_t(sx * sx + sz * sz)));
        finishPath(*path);
        int32_t cur = start;
        path->run.push_back(cur);
        // Р38: земля - не только на оси, но и поперёк русла: на склоне (река идёт вдоль склона, а не вниз) земля
        // ниже по склону в нескольких блоках сбоку уже гораздо ниже оси - вода по уровню оси стояла над ней стеной.
        // Минимум - на оси и по обе стороны на две ширины русла (с берегом): на склоне река врезается террасой.
        const int64_t across = 2 * 2 * (24 + s * 14 / 256) + 2 * VOXELS_PER_BLOCK;
        for (size_t k = 1; k < path->x.size(); ++k) {
            const size_t kn = std::min(k + 1, path->x.size() - 1);
            const int64_t tx = path->x[kn] - path->x[k - 1], tz = path->z[kn] - path->z[k - 1];
            const int64_t tl = std::max<int64_t>(1, int64_t(isqrt(uint64_t(tx * tx + tz * tz))));
            const int64_t nx = -tz * across / tl, nz = tx * across / tl;
            cur = std::min({cur, groundLine(p, path->x[k], path->z[k]), groundLine(p, path->x[k] + nx, path->z[k] + nz),
                            groundLine(p, path->x[k] - nx, path->z[k] - nz)});
            const int64_t u = path->cum[k];
            const int64_t us = path->len > 0 ? u * straight / path->len : 0;
            const int64_t byMouth = mdist > 0 ? int64_t(std::max(0, start - mlevel)) * us / mdist : 0;
            cur = std::min(cur, start - int32_t(std::max(byMouth, u / gradeDiv)));
            path->run.push_back(std::max(cur, p.seaLevel - 1));
        }
        // Р35: исток (водосбор - один свой узел) - ручеёк: ширина от четверти нарастает до полной за 96 блоков
        if (area(p, i, j) == 1) {
            path->wq.resize(path->x.size());
            for (size_t k = 0; k < path->x.size(); ++k)
                path->wq[k] = fx::q16(std::min<int64_t>(fx::ONE, fx::ONE / 4 + int64_t(fx::ONE) * 3 / 4 * path->cum[k] / (96 * VOXELS_PER_BLOCK)));
        }
    }
    finishPath(*path);
    area(p, i, j);   // узел - целиком: публикуем вместе с полноводностью
    Info& m = info(p, i, j);
    m.path = std::move(path);
    publish(p, nodeKey(i, j), m);
    return *m.path;
}

// шум вдоль реки в [-1, 1] (у каждого свойства - своё зерно)
inline fx::q16 vary(const TerrainParams& p, int32_t x, int32_t z, int cellLog2, uint32_t salt) {
    return gen::simplex2(x, z, cellLog2, p.seed ^ salt);
}

constexpr int NSTEP_LOG2 = 3;   // узлы шумов рек - через 8 вокселей

RiverSet::NoiseNode noiseNode(const TerrainParams& p, int32_t nx, int32_t nz) {
    const int32_t x = nx << NSTEP_LOG2, z = nz << NSTEP_LOG2;
    RiverSet::NoiseNode n;
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

// Р32: ближайшая точка русла: ребро e, доля f вдоль него (Q16), квадрат расстояния. Группы рёбер, чья рамка
// дальше уже найденного (или предела), не проверяются.
struct Hit { int64_t d2; int e; int64_t f; };
Hit nearestOnPath(const RiverPath& r, int64_t x, int64_t z, int64_t limit2) {
    Hit best{limit2, -1, 0};
    const size_t nv = r.x.size();
    if (nv == 1) {
        const int64_t dx = x - r.x[0], dz = z - r.z[0];
        const int64_t d2 = dx * dx + dz * dz;
        if (d2 < best.d2) best = {d2, 0, 0};
        return best;
    }
    for (size_t g = 0; g < r.boxes.size(); ++g) {
        const RiverPath::Box& b = r.boxes[g];
        const int64_t bx = std::max<int64_t>({0, b.x0 - x, x - b.x1}), bz = std::max<int64_t>({0, b.z0 - z, z - b.z1});
        if (bx * bx + bz * bz >= best.d2) continue;
        const size_t e1 = std::min(g * RiverPath::GROUP + RiverPath::GROUP, nv - 1);
        for (size_t e = g * RiverPath::GROUP; e < e1; ++e) {
            const int64_t ex = r.x[e + 1] - r.x[e], ez = r.z[e + 1] - r.z[e], px = x - r.x[e], pz = z - r.z[e];
            const int64_t len2 = ex * ex + ez * ez;
            int64_t t = 0;
            if (len2 > 0) t = std::clamp<int64_t>(((px * ex + pz * ez) << 16) / len2, 0, fx::ONE);
            const int64_t dx = px - ((ex * t) >> 16), dz = pz - ((ez * t) >> 16);
            const int64_t d2 = dx * dx + dz * dz;
            if (d2 < best.d2) best = {d2, int(e), t};
        }
    }
    return best;
}

// Р32: обход рёбер русла ближе предела: visit(ребро, квадрат расстояния до ребра, u) - u - место на русле (путь
// от начала, воксели), с которого берётся уровень для этого ребра. Расстояние - обычное (до отрезка): непрерывно.
// Уровень: на ребре - в проекции точки; ДО начала ребра - выше по течению, в проекции на продолжение ребра назад
// (у начала русла - уровень истока): долина нижнего участка не прорезает русло выше по течению ниже его воды
// (у ручья круче стенок она осушала бы русло), а функция остаётся непрерывной; за концом ребра - уровень конца.
// reach(low) - дальше какого расстояния участок с нижним уровнем low столбец точно не меняет; блоки групп и
// группы, чьи рамки дальше, не смотрятся.
template <class Reach, class Visit>
void forEdges(const RiverPath& r, int g0, int g1, int64_t x, int64_t z, Reach reach, Visit visit) {
    if (r.x.size() == 1) {
        const int64_t lim = reach(r.run[0]);
        const int64_t dx = x - r.x[0], dz = z - r.z[0];
        if (dx * dx + dz * dz < lim * lim) visit(0, dx * dx + dz * dz, 0);
        return;
    }
    const size_t nv = r.x.size();
    for (int g = g0; g <= g1; ++g) {
        if (g % RiverPath::SUPER == 0 || g == g0) {   // начало блока групп: весь блок дальше предела - мимо
            const size_t sbi = size_t(g / RiverPath::SUPER);
            const RiverPath::Box& sb = r.supers[sbi];
            const int64_t lim = reach(r.superLow[sbi]);
            const int64_t sx = std::max<int64_t>({0, sb.x0 - x, x - sb.x1}), sz = std::max<int64_t>({0, sb.z0 - z, z - sb.z1});
            if (sx * sx + sz * sz >= lim * lim) { g = (g / RiverPath::SUPER + 1) * RiverPath::SUPER - 1; continue; }
        }
        const RiverPath::Box& b = r.boxes[size_t(g)];
        const int64_t lim = reach(r.boxLow[size_t(g)]), limit2 = lim * lim;
        const int64_t bx = std::max<int64_t>({0, b.x0 - x, x - b.x1}), bz = std::max<int64_t>({0, b.z0 - z, z - b.z1});
        if (bx * bx + bz * bz >= limit2) continue;
        const size_t e1 = std::min(size_t(g) * RiverPath::GROUP + RiverPath::GROUP, nv - 1);
        for (size_t e = size_t(g) * RiverPath::GROUP; e < e1; ++e) {
            const int64_t ex = r.x[e + 1] - r.x[e], ez = r.z[e + 1] - r.z[e], px = x - r.x[e], pz = z - r.z[e];
            const int64_t len2 = ex * ex + ez * ez, el = r.cum[e + 1] - r.cum[e];
            const int64_t pr = px * ex + pz * ez;
            int64_t t = 0;
            if (len2 > 0) t = std::clamp<int64_t>((pr << 16) / len2, 0, fx::ONE);
            const int64_t dx = px - ((ex * t) >> 16), dz = pz - ((ez * t) >> 16);
            const int64_t d2 = dx * dx + dz * dz;
            if (d2 >= limit2) continue;
            const int64_t s = el > 0 ? pr / el : 0;   // проекция на ребро, воксели (до начала - меньше нуля)
            visit(e, d2, std::max<int64_t>(0, r.cum[e] + std::min(s, el)));
        }
    }
}

// Р32: уровень ступенями в блок: плёс - ровная вода, перекат - ступень вниз на блок; где земля под рекой падает
// круто - ступени идут часто (пороги, каскад), где обрывается - падают разом (водопад). Ступени - по значению
// уровня, а не по звену: на стыке звеньев и в слиянии разрыва нет. Уровни вида 16k-1 - уровень моря среди них.
inline int32_t stepLevel(int32_t l) { return ((l + 1) & ~(VOXELS_PER_BLOCK - 1)) - 1; }

// уровень (без ступеней) в точке u русла: между вершинами - линейно
int32_t runAt(const RiverPath& r, int64_t u) {
    if (r.run.size() == 1 || r.len <= 0) return r.run[0];
    u = std::clamp<int64_t>(u, 0, r.len);
    const size_t k = std::min<size_t>(size_t(std::upper_bound(r.cum.begin(), r.cum.end(), u) - r.cum.begin()), r.cum.size() - 1);
    const int64_t a = r.cum[k - 1], span = std::max<int64_t>(1, r.cum[k] - a);
    return r.run[k - 1] + int32_t(int64_t(r.run[k] - r.run[k - 1]) * (u - a) / span);
}

// Р33: впадение при встрече. Русла соседних звеньев могут сойтись (диагональные связи узлов перекрещиваются,
// русло отходит от прямой на ±400 блоков): ручей доходит до долины реки ниже себя - не через узел. Раньше
// долина реки прорезала землю под руслом ручья ниже его воды - вода ручья висела плитами и столбами. Теперь ручей
// там кончается - впадает: русло обрезается в первой своей вершине, где долина другой реки может опуститься
// ниже его воды больше чем на SPILL. Оценка долины - нижняя граница (пойма до 8 полуширин, стенки не положе
// 1:8, неровность -2 блока, дно на две глубины ниже уровня): обрезаем с запасом, вода до чужой долины не доходит
// висящей. Не обрезают: звенья с общим узлом (притоки, соседи по устью), своя цепочка вниз (петля самой реки -
// там минимум по рёбрам), реки не ниже ручья хотя бы на блок. Уровни и полноводность - прежние (без обрезки):
// сравнение - по необрезанным руслам, круга в расчёте нет.
std::shared_ptr<const RiverPath> drawPath(const TerrainParams& p, int32_t i, int32_t j) {
    if (const Info& n = info(p, i, j); n.trimmed) return n.drawPath;
    const RiverPath& base = outPath(p, i, j);
    std::shared_ptr<const RiverPath> result = info(p, i, j).path;
    const int d = down(p, i, j);
    if (d >= 0 && base.x.size() >= 2) {
        size_t cut = base.x.size();   // первая вершина, где ручей впадает (size - нет)
        std::shared_ptr<const RiverPath> into;   // Р35: в какую реку
        int64_t intoHalf = 0, intoDepth = 0;
        for (int32_t l = j - 3; l <= j + 3; ++l)
            for (int32_t k = i - 3; k <= i + 3; ++k) {
                if (k == i && l == j) continue;
                if (info(p, k, l).sea) continue;
                const int32_t a = area(p, k, l);
                const int od = down(p, k, l);
                if (a < (od >= 0 ? AREA_MIN : LAKE_AREA_MIN)) continue;
                if (od >= 0 && info(p, k, l).ti == i && info(p, k, l).tj == j) continue;   // свой приток - выше, не обрезает
                const RiverPath& o = outPath(p, k, l);
                const int64_t s = int64_t(isqrt(uint64_t(a) << 16));
                const int64_t oHalf = od >= 0 ? 24 + s * 14 / 256 : 256 + s * 64 / 256;
                const int64_t oDepth = od >= 0 ? 16 + s * 6 / 256 : 32 + s * 5 / 256;
                const int64_t oValley = oHalf + (od >= 0 ? 384 + s * 80 / 256 : 640);
                const int64_t reachMax = oValley + VALLEY_CAP;
                const RiverPath::Box& ob = o.bounds;
                const RiverPath::Box& bb = base.bounds;
                if (bb.x1 + reachMax < ob.x0 || bb.x0 - reachMax > ob.x1 || bb.z1 + reachMax < ob.z0 || bb.z0 - reachMax > ob.z1)
                    continue;
                const int32_t oLow = *std::min_element(o.run.begin(), o.run.end());
                for (size_t v = 0; v < cut; ++v) {
                    const int32_t lw = base.run[v];
                    if (oLow > lw - VOXELS_PER_BLOCK) continue;
                    // дальше этого долина другой реки ниже воды ручья больше чем на SPILL не опускается
                    const int64_t lim = std::min<int64_t>(reachMax, 8 * oHalf + 8 * (int64_t(lw) - SPILL - oLow + 32));
                    if (lim <= 0) continue;
                    const Hit h = nearestOnPath(o, base.x[v], base.z[v], lim * lim);
                    if (h.e < 0) continue;
                    const int32_t oRaw = o.run.size() == 1 ? o.run[0]
                                                           : o.run[size_t(h.e)] + int32_t((int64_t(o.run[size_t(h.e) + 1] - o.run[size_t(h.e)]) * h.f) >> 16);
                    if (oRaw > lw - VOXELS_PER_BLOCK) continue;   // не ниже ручья хотя бы на блок - слияние, не впадение
                    // дно чужой долины здесь: в русле (до двух полуширин) - на две глубины ниже уровня; в пойме (до
                    // 4 полуширин) - на блок выше; дальше - стенка 1:5 с неровностью до -2 блоков
                    const int64_t dd = int64_t(isqrt(uint64_t(h.d2)));
                    const int64_t bottom = dd < 2 * oHalf ? oRaw - 2 * oDepth
                                         : dd < 4 * oHalf ? oRaw + 1
                                                          : oRaw + 1 + (dd - 4 * oHalf) / 5 - 2 * VOXELS_PER_BLOCK;
                    if (bottom < lw - SPILL) {
                        cut = v;
                        into = info(p, k, l).path;
                        intoHalf = oHalf;
                        intoDepth = oDepth;
                        break;
                    }
                }
            }
        if (cut < base.x.size()) {
            // Р35: не обрыв, а ручеёк до русла реки. Последняя вершина ручья, где вода ещё не висит, - cut-1; от неё
            // прямо к ближайшей точке русла реки, вершины через 16 блоков. Уровень на продолжении спускается по дну
            // чужой долины (на блок ниже оценки дна), не ниже воды реки в месте впадения и только вниз - вода идёт
            // по склону каскадом и сливается с рекой. Ширина сужается к устью до трети - ручеёк, а не чаша.
            const RiverPath& o = *into;
            const size_t keep = std::max<size_t>(cut, 1);
            auto t = std::make_shared<RiverPath>();
            t->x.assign(base.x.begin(), base.x.begin() + keep);
            t->z.assign(base.z.begin(), base.z.begin() + keep);
            t->run.assign(base.run.begin(), base.run.begin() + keep);
            if (base.wq.size() == base.x.size()) t->wq.assign(base.wq.begin(), base.wq.begin() + keep);
            else t->wq.assign(keep, fx::ONE);
            const int64_t sx = t->x.back(), sz = t->z.back();
            const Hit join = nearestOnPath(o, sx, sz, INT64_MAX);
            if (join.e >= 0) {
                const auto oRawAt = [&](const Hit& h) {
                    return o.run.size() == 1 ? o.run[0]
                                             : o.run[size_t(h.e)] + int32_t((int64_t(o.run[size_t(h.e) + 1] - o.run[size_t(h.e)]) * h.f) >> 16);
                };
                int64_t qx = o.x[size_t(join.e)], qz = o.z[size_t(join.e)];
                if (o.x.size() > 1) {
                    qx += ((o.x[size_t(join.e) + 1] - o.x[size_t(join.e)]) * join.f) >> 16;
                    qz += ((o.z[size_t(join.e) + 1] - o.z[size_t(join.e)]) * join.f) >> 16;
                }
                const int32_t joinLevel = oRawAt(join);
                const int64_t ex = qx - sx, ez = qz - sz;
                const int64_t len = int64_t(isqrt(uint64_t(ex * ex + ez * ez)));
                const int n = int(std::max<int64_t>(1, (len + 128) / 256));
                int32_t lv = t->run.back();
                const fx::q16 w0 = t->wq.back();
                for (int k = 1; k <= n; ++k) {
                    const int64_t px = sx + ex * k / n, pz = sz + ez * k / n;
                    const Hit h = nearestOnPath(o, px, pz, INT64_MAX);
                    const int64_t dd = int64_t(isqrt(uint64_t(h.d2)));
                    const int32_t oRaw = oRawAt(h);
                    const int64_t bottom = dd < 2 * intoHalf ? oRaw - 2 * intoDepth
                                         : dd < 4 * intoHalf ? oRaw + 1
                                                             : oRaw + 1 + (dd - 4 * intoHalf) / 5 - 2 * VOXELS_PER_BLOCK;
                    lv = std::max(joinLevel, std::min<int32_t>(lv, int32_t(bottom - VOXELS_PER_BLOCK)));
                    if (k == n) lv = std::min(lv, joinLevel);
                    t->x.push_back(px);
                    t->z.push_back(pz);
                    t->run.push_back(std::max(lv, p.seaLevel - 1));
                    t->wq.push_back(w0 - fx::q16(int64_t(w0 - fx::ONE / 3) * k / n));
                }
            }
            finishPath(*t);
            result = std::move(t);
        }
    }
    Info& m = info(p, i, j);
    m.drawPath = result;
    m.trimmed = true;
    return result;
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
    const int32_t end = outPath(p, i, j).run.back();
    const Info& n = info(p, i, j);
    return {n.px, n.pz, n.pot, n.sea, d, a, lv, end, n.ti, n.tj};
}

RiverFlow riverFlow(const TerrainParams& p, int32_t i, int32_t j) {
    if (g_cache.params != &p || g_cache.seed != p.seed || g_cache.nodes.size() > 400000) {
        g_cache.nodes.clear();
        g_cache.params = &p;
        g_cache.seed = p.seed;
    }
    const int d = down(p, i, j);
    const Info& n = info(p, i, j);
    return {n.px, n.pz, n.pot, n.sea, d, n.ti, n.tj};
}

void RiverSet::build(const TerrainParams& p, int32_t x0, int32_t z0, int32_t x1, int32_t z1) {
    if (g_cache.params != &p || g_cache.seed != p.seed || g_cache.nodes.size() > 400000) {
        g_cache.nodes.clear();
        g_cache.params = &p;
        g_cache.seed = p.seed;
    }
    segs.clear();
    const int64_t margin = (SPILL_R + 1) * CELL;   // звено (Р37: сток котловины - до SPILL_R узлов), коридор, долина
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
            if (d >= 0) {   // река к соседу ниже
                g.half = int32_t(24 + s * 14 / 256);
                g.valley = g.half + int32_t(384 + s * 80 / 256);
                g.depth = int32_t(16 + s * 6 / 256);
            } else {        // котловина - озеро
                g.half = int32_t(256 + s * 64 / 256);
                g.valley = g.half + 640;
                g.depth = int32_t(32 + s * 5 / 256);
            }
            // грубо, до русла: задевает ли прямоугольник прямая между узлами с коридором поиска и меандрами
            const int64_t r = 2 * g.valley + VALLEY_CAP;
            {
                const Info& na = info(p, i, j);
                const Info& nb = d >= 0 ? info(p, na.ti, na.tj) : na;
                const int64_t rr = r + SPREAD + 3 * MEANDER / 2 + 2 * ANCHOR;
                if (std::max(na.px, nb.px) + rr < x0 || std::min(na.px, nb.px) - rr > x1 ||
                    std::max(na.pz, nb.pz) + rr < z0 || std::min(na.pz, nb.pz) - rr > z1)
                    continue;
            }
            g.path = d >= 0 ? drawPath(p, i, j) : (outPath(p, i, j), info(p, i, j).path);
            if (d < 0) {
                // озеро: уровень не выше самой низкой точки кольца берега - вода не переливается стеной
                int32_t rim = INT32_MAX;
                static constexpr int RX[8] = {2, 1, 0, -1, -2, -1, 0, 1}, RZ[8] = {0, 1, 2, 1, 0, -1, -2, -1};
                const int64_t cx = g.path->x[0], cz = g.path->z[0];
                for (int k = 0; k < 8; ++k)
                    rim = std::min(rim, groundLine(p, cx + RX[k] * g.half / 2, cz + RZ[k] * g.half / 2));
                auto lake = std::make_shared<RiverPath>(*g.path);
                lake->run = {std::max(std::min(lake->run[0], rim), p.seaLevel - 1)};
                g.path = std::move(lake);
            }
            const RiverPath::Box& b = g.path->bounds;
            if (b.x1 + r < x0 || b.x0 - r > x1 || b.z1 + r < z0 || b.z0 - r > z1) continue;
            g.len = g.path->len;
            // Р32: какие группы рёбер могут дотянуться до прямоугольника, и нижний уровень на них (а не на всём
            // звене: у горной реки он на километры ниже - отсечка по нему пропускала бы всё)
            {
                const RiverPath& P = *g.path;
                const int64_t reach = g.valley + VALLEY_CAP + 8 * int64_t(g.half);
                g.gFirst = INT32_MAX;
                g.gLast = -1;
                int32_t lo = INT32_MAX;
                for (size_t k = 0; k < P.boxes.size(); ++k) {
                    const RiverPath::Box& bb = P.boxes[k];
                    if (bb.x1 + reach < x0 || bb.x0 - reach > x1 || bb.z1 + reach < z0 || bb.z0 - reach > z1) continue;
                    g.gFirst = std::min(g.gFirst, int32_t(k));
                    g.gLast = int32_t(k);
                    const size_t v1 = std::min(k * RiverPath::GROUP + RiverPath::GROUP, P.run.size() - 1);
                    for (size_t v = k * RiverPath::GROUP; v <= v1; ++v) lo = std::min(lo, P.run[v]);
                }
                if (g.gLast < 0) continue;
                g.minLevel = lo;
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

int32_t RiverSet::levelAlong(const Seg& g, int64_t u) { return stepLevel(runAt(*g.path, u)); }

void RiverSet::apply(const TerrainParams& p, int32_t x, int32_t z, int32_t& h, int32_t& water, bool& shore) const {
    if (segs.empty()) return;
    const int32_t h0 = h;
    // дальше какого расстояния звено с уровнем воды не ниже low столбец точно не меняет (профиль ниже): пойма до
    // 8 полуширин звена, стенки не положе 1:8 (неровность до 2 блоков), не дальше края долины; берег и рукав - до
    // 8 полуширин и 4 блоков
    const auto reachOf = [&](const Seg& g, int32_t low) {
        const int64_t cut = int64_t(std::max(0, h0 - stepLevel(low))) + 3 * VOXELS_PER_BLOCK;
        return std::max<int64_t>(std::min<int64_t>(8 * int64_t(g.half) + 8 * cut, g.valley + VALLEY_CAP),
                                 8 * int64_t(g.half) + 4 * VOXELS_PER_BLOCK) + 1;
    };
    // Проход 1 (дёшево): задевает ли столбец хоть одно звено - шумы считаем, только если да
    bool any = false, braidable = false;
    for (const Seg& g : segs) {
        const int64_t r = reachOf(g, g.minLevel);
        const RiverPath::Box& b = g.path->bounds;
        if (x < b.x0 - r || x > b.x1 + r || z < b.z0 - r || z > b.z1 + r) continue;
        any = true;
        braidable |= g.area >= 20 && g.len > 0;
    }
    if (!any) return;

    // Шумы. Р30: медленные (ширина, глубина, пойма, стенки); Р31: местные - неровная кромка (32 блока),
    // бугры поймы (16 и 8 блоков), неровные берега и дно
    fx::q16 nv[N_COUNT];
    noiseAt(p, x, z, nv);   // Р31: шумы рек - из сетки узлов
    const fx::q16 nWidth = nv[N_WIDTH], nDepth = nv[N_DEPTH], nPlain = nv[N_PLAIN], nBank = nv[N_BANK];
    const fx::q16 nEdge = nv[N_EDGE], nBankL = nv[N_BANKL], nBed = nv[N_BED], b1 = nv[N_B1], b2 = nv[N_B2];
    const int32_t bump = int32_t(((int64_t(std::abs(b1)) * 2 + std::abs(b2)) * (2 * VOXELS_PER_BLOCK)) / (3 * int64_t(fx::ONE)));
    const fx::q16 kWidth = fx::mul(fx::ONE * 11 / 10 + nWidth / 2, fx::ONE + nEdge / 4);   // ×0.6..1.6, кромка ±25%
    const fx::q16 kDepth = fx::mul(fx::ONE + nDepth * 3 / 10, fx::ONE + nBed / 5);         // ±30%, дно ±20%
    const fx::q16 kPlain = fx::ONE * 7 / 4 + nPlain * 5 / 4;                               // пойма ×0.5..3
    // стенки долины 1:2..1:8 - заложение bankRun (Q16, непрерывно: целый делитель прыгал бы). Р32: местная
    // неровность стенок - добавкой не больше 2 блоков, а не множителем заложения: множитель на глубокой прорези
    // раскачивал стену на десятки блоков за 32 блока (уклон до 4:1)
    const int64_t bankRun = 5 * int64_t(fx::ONE) + 3 * int64_t(nBank);
    const int64_t invBank = (int64_t(fx::ONE) << fx::SHIFT) / bankRun;   // 1/заложение, Q16
    // Р31: рукава - второе русло у больших рек на ровных участках. Р32: точка запроса для рукава сдвигается
    // шумом на несколько ширин русла (рукав расходится с основным и сходится снова); только на ровном - сдвиг
    // вдоль реки там уровня не меняет
    const fx::q16 braid = braidable ? fx::smoothstep(fx::ONE * 15 / 100, fx::ONE * 35 / 100, nv[N_BRAID]) : 0;

    // Р32: прорезь звена - минимум по всем его рёбрам высоты, которую дало бы каждое ребро (профиль долины с
    // маской к краю). Минимум непрерывных функций непрерывен: у петли, у крутого ручья, на стыке рёбер нет
    // перескоков (сменилось ближайшее ребро - высота нет).
    struct Cut { int32_t h, lw; bool wet; };
    Cut cuts[64];
    int nCuts = 0;
    for (const Seg& g : segs) {
        if (nCuts == 64) break;
        const RiverPath& path = *g.path;
        const int64_t rup = reachOf(g, g.minLevel);
        const RiverPath::Box& b = path.bounds;
        if (x < b.x0 - rup || x > b.x1 + rup || z < b.z0 - rup || z > b.z1 + rup) continue;
        const int32_t half = std::max<int32_t>(16, int32_t((int64_t(g.half) * (g.len > 0 ? kWidth : fx::ONE)) >> fx::SHIFT));
        const int32_t depth = int32_t((int64_t(g.depth) * kDepth) >> fx::SHIFT);
        // высота от ребра: профиль - в русле дно чашей; пойма на блок выше воды, бугры нарастают от кромки за
        // 3 блока; дальше стенки с заложением bankRun и неровностью. Маска: только прорезает, к краю долины -
        // исходный рельеф; ширина долины - по глубине прорези (на равнине широкая пологая, в горах V-образная,
        // не шире VALLEY_CAP); считается от уровня без ступеней (от ступенчатого прыгала бы на перекатах)
        const auto carve = [&](int64_t d, int32_t lw, int32_t raw, int32_t half, int32_t depth) -> int64_t {
            const fx::q16 nearSea = fx::smoothstep(p.seaLevel, p.seaLevel + 8 * VOXELS_PER_BLOCK, lw);
            const int64_t plain = half + ((int64_t(half) * fx::mul(kPlain, nearSea)) >> fx::SHIFT);
            const int64_t radius = std::clamp<int64_t>(plain + ((bankRun * std::max(0, h0 - raw)) >> fx::SHIFT),
                                                       g.valley, g.valley + VALLEY_CAP);
            if (d >= radius) return h0;
            int64_t prof;
            const int32_t dc = int32_t(d);
            if (d < half) prof = lw - depth + int64_t(depth) * d * d / (int64_t(half) * half);
            else {
                const int64_t bumpHere = (int64_t(bump) * fx::smoothstep(half, half + 3 * VOXELS_PER_BLOCK, dc)) >> fx::SHIFT;
                // пойма и стенки - от уровня без ступеней: гладкие, без террас поперёк поймы на каждом перекате
                // (ступени - только у воды и дна русла; на кромке русла берег до блока)
                if (d < plain) prof = raw + 1 + bumpHere;
                else {
                    const int64_t wall = ((d - plain) * invBank) >> fx::SHIFT;
                    const int64_t rough = std::min<int64_t>(2 * VOXELS_PER_BLOCK, (d - plain) / 8);   // от кромки поймы
                    prof = raw + 1 + bumpHere + wall + ((int64_t(nBankL) * rough) >> fx::SHIFT);
                }
            }
            const fx::q16 w = fx::ONE - fx::smoothstep(int32_t(radius * 6 / 10), int32_t(radius), dc);
            return h0 + ((int64_t(w) * std::min<int64_t>(0, prof - h0)) >> fx::SHIFT);
        };
        int64_t best = INT64_MAX, bestD = INT64_MAX;
        int32_t bestLw = 0, bestHalf = half;
        bool nearShore = false;
        const bool narrow = path.wq.size() == path.x.size();
        const auto visit = [&](size_t e, int64_t d2, int64_t u) {
            const int64_t d = int64_t(isqrt(uint64_t(d2)));
            int32_t raw;
            fx::q16 wq = fx::ONE;
            if (path.run.size() == 1) raw = path.run[0];
            else if (u >= path.cum[e]) {   // на ребре - между его вершинами
                const int64_t el = std::max<int64_t>(1, path.cum[e + 1] - path.cum[e]);
                raw = path.run[e] + int32_t(int64_t(path.run[e + 1] - path.run[e]) * (u - path.cum[e]) / el);
                if (narrow) wq = path.wq[e] + fx::q16(int64_t(path.wq[e + 1] - path.wq[e]) * (u - path.cum[e]) / el);
            } else {                       // до начала ребра - выше по течению
                raw = runAt(path, u);
                if (narrow) wq = path.wq[e];
            }
            // Р35: ширина и глубина русла - по ребру (у истока и у впадения - ручеёк)
            const int32_t hE = narrow ? std::max<int32_t>(8, int32_t((int64_t(half) * wq) >> fx::SHIFT)) : half;
            const int32_t dE = narrow ? std::max<int32_t>(8, int32_t((int64_t(depth) * (fx::ONE + wq) / 2) >> fx::SHIFT)) : depth;
            nearShore |= d < hE + 2 * VOXELS_PER_BLOCK;
            const int32_t lw = stepLevel(raw);
            const int64_t f = carve(d, lw, raw, hE, dE);
            if (f < best || (f == best && d < bestD)) { best = f; bestD = d; bestLw = lw; bestHalf = hE; }
        };
        forEdges(path, g.gFirst, g.gLast, x, z, [&](int32_t low) { return reachOf(g, low); }, visit);
        if (best == INT64_MAX || (best >= h0 && !nearShore && braid == 0)) continue;
        bool wet = bestD < bestHalf;
        // рукав: у рек от 20 узлов, только на ровном (вода у земли) - второе русло той же глубины и уровня,
        // вокруг него откос 1:1 (через минимум с основным профилем - без ступенек)
        if (braid > 0 && g.area >= 20 && g.len > 0) {
            const fx::q16 flat = fx::ONE - fx::smoothstep(0, 4 * VOXELS_PER_BLOCK, h0 - bestLw);
            const int32_t half2 = int32_t((int64_t(half) * 6 / 10 * fx::mul(braid, flat)) >> fx::SHIFT);
            if (half2 > 8) {
                const int64_t shift = 6 * int64_t(g.half);
                const int64_t qx2 = x + ((int64_t(nv[N_WX2]) * shift) >> fx::SHIFT);
                const int64_t qz2 = z + ((int64_t(nv[N_WZ2]) * shift) >> fx::SHIFT);
                const int64_t outer = int64_t(half2) + 4 * VOXELS_PER_BLOCK;
                const Hit h2 = nearestOnPath(path, qx2, qz2, outer * outer);
                if (h2.e >= 0) {
                    const int64_t d2b = int64_t(isqrt(uint64_t(h2.d2)));
                    const int64_t pb = d2b < half2 ? bestLw - depth + int64_t(depth) * d2b * d2b / (int64_t(half2) * half2)
                                                   : bestLw + (d2b - half2);
                    best = std::min(best, pb);   // у русла маска - 1: рукав прорезает прямо
                    wet |= d2b < half2;
                }
            }
        }
        const int32_t hc = int32_t(std::min<int64_t>(h0, best));
        h = std::min(h, hc);
        cuts[nCuts++] = {hc, bestLw, wet};
        if (nearShore) shore = true;
    }
    // Р32: вода русла. Вала больше нет: уровень по построению не выше земли на русле (остаток - рябь до блока).
    // Но долина ДРУГОЙ реки может прорезать землю под этим руслом ниже его воды (ручей идёт по дну чужой долины,
    // приток - по пойме главной реки; своё другое плечо у петли уже учтено минимумом по рёбрам). Тогда вода здесь
    // не держится: если поверхность от других звеньев (их прорезь или их вода) ниже уровня этого русла больше чем
    // на SPILL - русло здесь сухое, вода сливается в долину ниже (маленький водопад на краю), а не висит столбом.
    for (int k = 0; k < nCuts; ++k) {
        if (!cuts[k].wet) continue;
        int32_t other = INT32_MAX;
        for (int j = 0; j < nCuts; ++j)
            if (j != k) other = std::min(other, cuts[j].wet ? std::max(cuts[j].h, cuts[j].lw) : cuts[j].h);
        if (other < cuts[k].lw - SPILL) continue;
        water = std::max(water, cuts[k].lw);
    }
}

RiverSet::Current RiverSet::current(const TerrainParams& p, int32_t x, int32_t z) const {
    // Р31: течение в русле - вдоль русла (от истока к устью); скорость - как в гидравлике (Маннинг):
    // растёт с глубиной (полноводность) и с уклоном воды (крутизна): плёс - медленно, порог - быстро,
    // водопад - предел. v = 8 · √(уклон) · √(глубина в блоках) блоков/с, от 0.2 до 12. Р32: уклон - по уровню
    // без ступеней на 8 блоках пути (средний уклон участка, а не ноль на плёсе и бесконечность на ступени).
    (void)p;
    Current c{};
    int64_t best = INT64_MAX;
    for (const Seg& g : segs) {
        if (g.len <= 0) continue;
        const RiverPath& path = *g.path;
        const int64_t lim = std::min<int64_t>(best, int64_t(g.half) * g.half * 3);   // в русле (с запасом на ширину)
        const RiverPath::Box& b = path.bounds;
        if (x < b.x0 - 2 * g.half || x > b.x1 + 2 * g.half || z < b.z0 - 2 * g.half || z > b.z1 + 2 * g.half) continue;
        const Hit hit = nearestOnPath(path, x, z, lim);
        if (hit.e < 0) continue;
        best = hit.d2;
        const size_t e = size_t(hit.e);
        const int64_t ex = path.x[e + 1] - path.x[e], ez = path.z[e + 1] - path.z[e];
        const int64_t elen = std::max<int64_t>(1, path.cum[e + 1] - path.cum[e]);
        const int64_t u = path.cum[e] + ((elen * hit.f) >> 16), step = 8 * VOXELS_PER_BLOCK;
        const int32_t l0 = runAt(path, u), l1 = runAt(path, u + step);
        const int64_t slope = (int64_t(std::max(0, l0 - l1)) << fx::SHIFT) / step;   // Q16, вокселей на воксель
        const int64_t depthBlocks = std::max<int64_t>(1, g.depth / VOXELS_PER_BLOCK);
        int64_t v = 8 * int64_t(isqrt(uint64_t(slope) << 16)) * int64_t(isqrt(uint64_t(depthBlocks) << 32)) >> 16;   // Q16
        v = std::clamp<int64_t>(v, fx::ONE / 5, 12 * int64_t(fx::ONE));
        c.dirX = fx::q16((ex << fx::SHIFT) / elen);
        c.dirZ = fx::q16((ez << fx::SHIFT) / elen);
        c.speed = fx::q16(v);
        c.inRiver = true;
    }
    return c;
}

}  // namespace world
