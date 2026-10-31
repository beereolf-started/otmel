#include "world/lod.h"

#include <algorithm>
#include <cmath>

#include "world/caves.h"
#include "world/forest.h"
#include "world/rivers.h"

namespace world {
namespace {

// наклон (разность высот через 2 шага) в 1/256 вокселя на воксель, с округлением, в пределах int16
inline uint32_t slope256(int32_t dh, int64_t stepVoxels) {
    const int64_t num = int64_t(dh) * 256, den = 2 * stepVoxels;
    const int64_t g = (num >= 0 ? num + den / 2 : num - den / 2) / den;
    return uint32_t(uint16_t(int16_t(std::clamp<int64_t>(g, -32767, 32767))));
}
inline uint32_t h16(int32_t h) { return uint32_t(uint16_t(int16_t(std::clamp(h, -32768, 32767)))); }

// Р34: поверхность воды в узлах [0..n) сетки с каймой в узел (n + 2 по стороне): у мокрого - его вода; у сухого
// рядом с мокрым (8 соседей) - наибольшая соседняя вода, но не выше дна без вокселя; у сухого без воды рядом -
// блок под дном (его треугольники целиком под землёй). Правило одно для обоих уровней: сползший узел = родитель.
template <int N>
void waterSurface(const int32_t (&h)[N][N], const int32_t (&w)[N][N], const bool (&wet)[N][N], int32_t (&out)[N][N]) {
    for (int j = 1; j < N - 1; ++j)
        for (int i = 1; i < N - 1; ++i) {
            if (wet[j][i]) { out[j][i] = w[j][i]; continue; }
            int32_t best = INT32_MIN;
            for (int dj = -1; dj <= 1; ++dj)
                for (int di = -1; di <= 1; ++di)
                    if (wet[j + dj][i + di]) best = std::max(best, w[j + dj][i + di]);
            out[j][i] = best == INT32_MIN ? h[j][i] - VOXELS_PER_BLOCK : std::min(best, h[j][i] - 1);
        }
}

}  // namespace

void buildTile(const TerrainParams& p, TileKey k, TileMesh& out) {
    const int L = k.level;
    const int64_t step = int64_t(VOXELS_PER_BLOCK) << L;            // шаг узлов, воксели
    const int64_t x0 = int64_t(k.tx) * TILE * step, z0 = int64_t(k.tz) * TILE * step;
    // октавы мельче двух шагов не различимы: уровень L - ячейка от 2^(L+5) вокселей, уровень L+1 - от 2^(L+6)
    const int fineCut = L + 5, coarseCut = L + 6;

    // медленные поля (Р22): на мелких уровнях узлы сетки общие у многих вершин - считаем их один раз;
    // с шага 64 вокселя вершины сами стоят на узлах - каждая спрашивает ровно один узел
    static thread_local SlowGrid grid;
    const bool dense = step < (int64_t(1) << SLOW_STEP_LOG2);
    if (dense) grid.build(p, int32_t(x0 - 2 * step), int32_t(z0 - 2 * step), int32_t(x0 + 34 * step), int32_t(z0 + 34 * step), fineCut);
    const auto slowFine = [&](int64_t x, int64_t z) { return dense ? grid.at(int32_t(x), int32_t(z)) : slowAt(p, int32_t(x), int32_t(z), fineCut); };

    // Р29: звенья рек и озёра, задевающие плитку (с каймой), - один раз на плитку
    static thread_local RiverSet rivers;
    rivers.build(p, int32_t(x0 - 2 * step), int32_t(z0 - 2 * step), int32_t(x0 + 34 * step), int32_t(z0 + 34 * step));

    // высоты своего уровня с каймой в один узел (для наклона): [-1..33]. Р34: высота - дно (вода - своим слоем,
    // прозрачным); покров дна - как у вокселей: у воды рек и озёр - песок (Р29), иначе по климату
    static thread_local int32_t hf[TILE_VERTS + 2][TILE_VERTS + 2], wf[TILE_VERTS + 2][TILE_VERTS + 2];
    static thread_local bool wetF[TILE_VERTS + 2][TILE_VERTS + 2], shoreF[TILE_VERTS + 2][TILE_VERTS + 2];
    for (int j = -1; j <= TILE_VERTS; ++j)
        for (int i = -1; i <= TILE_VERTS; ++i) {
            const int64_t x = x0 + i * step, z = z0 + j * step;
            const Relief r = reliefAt(p, int32_t(x), int32_t(z), slowFine(x, z), fineCut, &rivers);
            const int32_t w = waterLevelAt(p, r);
            hf[j + 1][i + 1] = r.height;
            wf[j + 1][i + 1] = w;
            wetF[j + 1][i + 1] = w > r.height;
            shoreF[j + 1][i + 1] = r.shore;
        }
    // высоты уровня L+1 в чётных узлах (шаг вдвое крупнее) с каймой: [-1..17]
    constexpr int CV = TILE / 2 + 1;
    static thread_local int32_t hc[CV + 2][CV + 2], wc[CV + 2][CV + 2];
    static thread_local bool wetC[CV + 2][CV + 2], shoreC[CV + 2][CV + 2];
    for (int j = -1; j <= CV; ++j)
        for (int i = -1; i <= CV; ++i) {
            const int32_t cx = int32_t(x0 + i * 2 * step), cz = int32_t(z0 + j * 2 * step);
            const Relief r = reliefAt(p, cx, cz, slowAt(p, cx, cz, coarseCut), coarseCut, &rivers);
            const int32_t w = waterLevelAt(p, r);
            hc[j + 1][i + 1] = r.height;
            wc[j + 1][i + 1] = w;
            wetC[j + 1][i + 1] = w > r.height;
            shoreC[j + 1][i + 1] = r.shore;
        }
    // наклон и покров уровня L+1 - в чётных узлах
    static thread_local uint32_t coarseGrad[CV][CV], coarseCover[CV][CV];
    for (int j = 0; j < CV; ++j)
        for (int i = 0; i < CV; ++i) {
            coarseGrad[j][i] = slope256(hc[j + 1][i + 2] - hc[j + 1][i], 2 * step) |
                               slope256(hc[j + 2][i + 1] - hc[j][i + 1], 2 * step) << 16;
            const int32_t wx = int32_t(x0 + i * 2 * step), wz = int32_t(z0 + j * 2 * step);
            Cover c = shoreC[j + 1][i + 1] ? Cover{SAND, SAND}
                                             : coverAt(p, climateAt(p, wx, wz, slowFine(wx, wz)), hc[j + 1][i + 1]);   // климату отсечка октав не нужна
            // Р41: устье пещеры - бурым пятном (вдали пещер нет - только метка); шаг грубой сетки - вдвое
            if (!shoreC[j + 1][i + 1] && !wetC[j + 1][i + 1] && caveMouth(p, wx, wz, hc[j + 1][i + 1], int32_t(2 * step / VOXELS_PER_BLOCK)))
                c = Cover{DIRT, DIRT};
            coarseCover[j][i] = uint32_t(c.top) | uint32_t(c.under) << 4;
        }

    static thread_local int32_t sf[TILE_VERTS + 2][TILE_VERTS + 2], sc[CV + 2][CV + 2];
    waterSurface(hf, wf, wetF, sf);
    waterSurface(hc, wc, wetC, sc);

    out.data.resize(size_t(TILE_VERTS) * TILE_VERTS * TILE_WORDS);
    out.falls.clear();
    out.minH = INT32_MAX;
    out.maxH = INT32_MIN;
    out.water = false;
    for (int j = 0; j < TILE_VERTS; ++j)
        for (int i = 0; i < TILE_VERTS; ++i) {
            const int32_t h = hf[j + 1][i + 1];
            // куда сползает узел: нечётный номер - к меньшему чётному соседу, т.е. в чётный узел (i/2, j/2)
            const int ci = i >> 1, cj = j >> 1;
            const int32_t hcT = hc[cj + 1][ci + 1];

            const Climate cl = climateAt(p, int32_t(x0 + i * step), int32_t(z0 + j * step), slowFine(x0 + i * step, z0 + j * step));
            Cover c = shoreF[j + 1][i + 1] ? Cover{SAND, SAND} : coverAt(p, cl, h);
            // Р67: густота леса в узле (как у дубов вблизи и спрайтов) - для шума крон вдали; у воды, на берегу и на
            // кручах (больше блока на блок) - ноль
            float forest = 0.0f;
            if (!shoreF[j + 1][i + 1] && !wetF[j + 1][i + 1] &&
                std::max(std::abs(hf[j + 1][i + 2] - hf[j + 1][i]), std::abs(hf[j + 2][i + 1] - hf[j][i + 1])) <= 2 * step)
                forest = forestDensity(p, int32_t(x0 + i * step), int32_t(z0 + j * step), h, cl);
            if (!shoreF[j + 1][i + 1] && !wetF[j + 1][i + 1] &&
                caveMouth(p, int32_t(x0 + i * step), int32_t(z0 + j * step), h, int32_t(step / VOXELS_PER_BLOCK)))
                c = Cover{DIRT, DIRT};
            uint32_t* w = &out.data[(size_t(j) * TILE_VERTS + i) * TILE_WORDS];
            w[0] = h16(h) | h16(hcT) << 16;
            w[1] = slope256(hf[j + 1][i + 2] - hf[j + 1][i], step) | slope256(hf[j + 2][i + 1] - hf[j][i + 1], step) << 16;
            w[2] = coarseGrad[cj][ci];
            w[3] = uint32_t(c.top) | uint32_t(c.under) << 4 | coarseCover[cj][ci] << 8 | uint32_t(std::lround(forest * 255.0f)) << 16;
            const int32_t s = sf[j + 1][i + 1], scT = sc[cj + 1][ci + 1];
            // Р75: течение у мокрого узла - чтобы и вдали было видно, куда и как быстро течёт река; перепад глади к
            // самому низкому мокрому соседу - пена у падения и места брызг вдали
            if (wetF[j + 1][i + 1]) {
                const int64_t wx = x0 + i * step, wz = z0 + j * step;
                const RiverSet::Current cur = rivers.current(p, int32_t(wx), int32_t(wz));
                uint32_t flow = 0;
                if (cur.inRiver) {
                    const float sp = float(cur.speed) / float(fx::ONE);
                    const float ang = std::atan2(float(cur.dirZ), float(cur.dirX));
                    flow = uint32_t(std::lround(ang / 6.2831853f * 16.0f)) & 15u;
                    flow |= uint32_t(sp < 0.5f ? 0 : sp < 1.6f ? 1 : sp < 3.0f ? 2 : 3) << 4;
                }
                int32_t drop = 0, bi = 0, bj = 0;
                for (int dj = -1; dj <= 1; ++dj)
                    for (int di = -1; di <= 1; ++di) {
                        if ((di == 0 && dj == 0) || !wetF[j + 1 + dj][i + 1 + di]) continue;
                        // по уровню воды wf: у мокрых узлов он равен глади, а у каймы (за краем плитки) гладь sf не
                        // считается - там лежал мусор прошлой плитки, и края плиток в океане становились «водопадами»
                        const int32_t d = wf[j + 1][i + 1] - wf[j + 1 + dj][i + 1 + di];
                        if (d > drop) { drop = d; bi = di; bj = dj; }
                    }
                flow |= uint32_t(drop < 2 ? 0 : drop < 5 ? 1 : drop < 12 ? 2 : 3) << 6;
                w[3] |= flow << 24;
                // своя кромка - только у узлов плитки без общего края с соседней плиткой (там - её узлы)
                if (i < TILE && j < TILE && drop >= std::max<int64_t>(2, step / 8)) {
                    const float len = std::sqrt(float(bi * bi + bj * bj));
                    out.falls.insert(out.falls.end(), {float(wx + bi * step / 2), float(s + 1), float(wz + bj * step / 2),
                                                       float(drop), float(bi) / len, float(bj) / len});
                }
            }
            w[4] = h16(s) | h16(scT) << 16;
            const bool wet = wetF[j + 1][i + 1] || wetC[cj + 1][ci + 1];
            out.water |= wet;
            out.minH = std::min({out.minH, h, hcT});
            out.maxH = std::max({out.maxH, h, hcT});
            if (wet) out.maxH = std::max({out.maxH, s, scT});
        }
}

int lodTopLevel(const LodSettings& s) {
    int L = 0;
    while (lodRange(s, L) < s.drawDistance && L < 20) ++L;
    return L;
}

namespace {

struct Selector {
    const LodSettings& s;
    double cx, cz;
    std::vector<TileDraw>& out;

    // расстояние по горизонтали от камеры до квадрата
    double dist(int64_t x0, int64_t z0, int64_t size) const {
        const double dx = std::max({double(x0) - cx, 0.0, cx - double(x0 + size)});
        const double dz = std::max({double(z0) - cz, 0.0, cz - double(z0 + size)});
        return std::sqrt(dx * dx + dz * dz);
    }
    bool insideHole(int64_t x0, int64_t z0, int64_t size) const {
        return x0 >= s.holeMinX && z0 >= s.holeMinZ && x0 + size <= s.holeMaxX && z0 + size <= s.holeMaxZ;
    }
    void emit(TileKey k, uint8_t mask) {
        const int64_t half = tileBlocks(k.level) / 2;
        const int64_t x0 = int64_t(k.tx) * tileBlocks(k.level), z0 = int64_t(k.tz) * tileBlocks(k.level);
        for (int q = 0; q < 4; ++q)
            if (mask >> q & 1) {
                const int64_t qx = x0 + (q & 1) * half, qz = z0 + (q >> 1) * half;
                if (insideHole(qx, qz, half) || dist(qx, qz, half) > s.drawDistance) mask &= uint8_t(~(1 << q));
            }
        if (mask) out.push_back({k, mask});
    }
    // true - место узла обработано (нарисовано им, его детьми или лежит вне видимости);
    // false - узел вне дальности своего уровня, его место рисует родитель своей четвертью
    bool select(TileKey k) {
        const int64_t size = tileBlocks(k.level);
        const double d = dist(int64_t(k.tx) * size, int64_t(k.tz) * size, size);
        if (d > s.drawDistance) return true;
        if (d > lodRange(s, k.level)) return false;
        if (k.level == 0 || d > lodRange(s, k.level - 1)) { emit(k, 0xF); return true; }
        uint8_t mask = 0;
        for (int q = 0; q < 4; ++q)
            if (!select({k.level - 1, k.tx * 2 + (q & 1), k.tz * 2 + (q >> 1)})) mask |= uint8_t(1 << q);
        if (mask) emit(k, mask);
        return true;
    }
};

}  // namespace

void selectTiles(const LodSettings& s, double camX, double camZ, std::vector<TileDraw>& out) {
    out.clear();
    const int top = lodTopLevel(s);
    const int64_t size = tileBlocks(top);
    const auto cell = [&](double v) { return int32_t(std::floor(v / double(size))); };
    Selector sel{s, camX, camZ, out};
    for (int32_t tz = cell(camZ - s.drawDistance); tz <= cell(camZ + s.drawDistance); ++tz)
        for (int32_t tx = cell(camX - s.drawDistance); tx <= cell(camX + s.drawDistance); ++tx)
            if (!sel.select({top, tx, tz})) sel.emit({top, tx, tz}, 0xF);
}

}  // namespace world
