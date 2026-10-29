#include "world/lod.h"

#include <algorithm>
#include <cmath>

namespace world {
namespace {

// наклон (разность высот через 2 шага) в 1/16 вокселя на воксель, с округлением, в пределах int8
inline int32_t slope16(int32_t dh, int64_t stepVoxels) {
    const int64_t num = int64_t(dh) * 16, den = 2 * stepVoxels;
    const int64_t g = (num >= 0 ? num + den / 2 : num - den / 2) / den;
    return int32_t(std::clamp<int64_t>(g, -127, 127));
}
inline uint32_t h16(int32_t h) { return uint32_t(uint16_t(int16_t(std::clamp(h, -32768, 32767)))); }

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

    // высоты своего уровня с каймой в один узел (для наклона): [-1..33]
    static thread_local int32_t hf[TILE_VERTS + 2][TILE_VERTS + 2];
    for (int j = -1; j <= TILE_VERTS; ++j)
        for (int i = -1; i <= TILE_VERTS; ++i) {
            const int64_t x = x0 + i * step, z = z0 + j * step;
            hf[j + 1][i + 1] = reliefAt(p, int32_t(x), int32_t(z), slowFine(x, z), fineCut).height;
        }
    // высоты уровня L+1 в чётных узлах (шаг вдвое крупнее) с каймой: [-1..17]
    constexpr int CV = TILE / 2 + 1;
    static thread_local int32_t hc[CV + 2][CV + 2];
    for (int j = -1; j <= CV; ++j)
        for (int i = -1; i <= CV; ++i)
            hc[j + 1][i + 1] = reliefAt(p, int32_t(x0 + i * 2 * step), int32_t(z0 + j * 2 * step), coarseCut).height;
    // наклон и покров уровня L+1 - в чётных узлах
    static thread_local uint32_t coarseWord1[CV][CV], coarseCover[CV][CV];
    for (int j = 0; j < CV; ++j)
        for (int i = 0; i < CV; ++i) {
            const int32_t gx = slope16(hc[j + 1][i + 2] - hc[j + 1][i], 2 * step);
            const int32_t gz = slope16(hc[j + 2][i + 1] - hc[j][i + 1], 2 * step);
            coarseWord1[j][i] = uint32_t(uint8_t(int8_t(gx))) | uint32_t(uint8_t(int8_t(gz))) << 8;
            const int32_t wx = int32_t(x0 + i * 2 * step), wz = int32_t(z0 + j * 2 * step);
            const Cover c = coverAt(p, climateAt(p, wx, wz, slowFine(wx, wz)), hc[j + 1][i + 1]);   // климату отсечка октав не нужна
            coarseCover[j][i] = uint32_t(c.top) | uint32_t(c.under) << 4;
        }

    out.data.resize(size_t(TILE_VERTS) * TILE_VERTS * TILE_WORDS);
    out.minH = INT32_MAX;
    out.maxH = INT32_MIN;
    for (int j = 0; j < TILE_VERTS; ++j)
        for (int i = 0; i < TILE_VERTS; ++i) {
            const int32_t h = hf[j + 1][i + 1];
            // куда сползает узел: нечётный номер - к меньшему чётному соседу, т.е. в чётный узел (i/2, j/2)
            const int ci = i >> 1, cj = j >> 1;
            const int32_t hcT = hc[cj + 1][ci + 1];
            const int32_t gx = slope16(hf[j + 1][i + 2] - hf[j + 1][i], step);
            const int32_t gz = slope16(hf[j + 2][i + 1] - hf[j][i + 1], step);
            const Cover c = coverAt(p, climateAt(p, int32_t(x0 + i * step), int32_t(z0 + j * step), slowFine(x0 + i * step, z0 + j * step)), h);
            uint32_t* w = &out.data[(size_t(j) * TILE_VERTS + i) * TILE_WORDS];
            w[0] = h16(h) | h16(hcT) << 16;
            w[1] = uint32_t(uint8_t(int8_t(gx))) | uint32_t(uint8_t(int8_t(gz))) << 8 | coarseWord1[cj][ci] << 16;
            w[2] = uint32_t(c.top) | uint32_t(c.under) << 4 | coarseCover[cj][ci] << 8;
            out.minH = std::min({out.minH, h, hcT});
            out.maxH = std::max({out.maxH, h, hcT});
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
