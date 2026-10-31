#include "world/mesher.h"

#include <algorithm>
#include <bit>
#include <cstring>

#include "world/bits.h"

namespace world {
namespace {

constexpr uint64_t ALL34 = (uint64_t(1) << 34) - 1;

// Маска вертикального столбца ФОРМУЛОЙ (Р18): у нетронутого рельефа твёрдо всё не выше поверхности top -
// сплошные единицы снизу. Бит i - воксель y = y0 - 1 + i (кайма снизу и сверху). n единиц снизу = 2^n - 1.
inline uint64_t columnFromHeight(int32_t top, int32_t y0) {
    const int64_t n = int64_t(top) - (y0 - 1) + 1;
    if (n <= 0) return 0;
    if (n >= 34) return ALL34;
    return (uint64_t(1) << n) - 1;
}

// Оси граней: для оси d плоскость - оси (u, v) с u × v = +d (правая тройка), обход p0 → +u·w → +u·w+v·h → +v·h
// идёт против часовой стрелки снаружи.   X: u = y, v = z      Y: u = z, v = x      Z: u = x, v = y
struct Axis { int d, u, v; };
constexpr Axis AXES[3] = {{0, 1, 2}, {1, 2, 0}, {2, 0, 1}};

inline uint32_t pack(int x, int y, int z, int normal, Material m) {
    return uint32_t(x) | uint32_t(y) << 6 | uint32_t(z) << 12 | uint32_t(normal) << 18 | uint32_t(m) << 21;
}

struct MatRows { Material mat; uint32_t rows[CHUNK]; };

}  // namespace

namespace {

// Видимые грани одного вида вокселей (self) - битами по трём осям, слияние в прямоугольники.
// Грань видна, где воксель self, а сосед в сторону грани не self и не block: у грунта block нет (дно под
// водой видно), у воды block - грунт (у воды остаются только грани в воздух). mat(x, y, z) - материал грани.
template <class MatFn>
void emitFaces(const uint64_t (&self)[3][CHUNK][CHUNK], const uint64_t (*block)[3][CHUNK][CHUNK], MatFn mat,
               std::vector<uint32_t>& out, int& quads) {
    uint32_t t[CHUNK];
    static thread_local uint32_t layerRows[CHUNK][CHUNK];   // [слой d][строка v], бит u
    static thread_local MatRows mats[64];
    for (int ai = 0; ai < 3; ++ai) {
        const Axis ax = AXES[ai];
        for (int sign = 0; sign < 2; ++sign) {
            const int normal = ai * 2 + sign;              // 0 +X, 1 -X, 2 +Y, 3 -Y, 4 +Z, 5 -Z
            // видимые грани столбца - одной операцией для всех 32 вокселей; затем поворот таблицы [u][биты d] ->
            // [d][биты u] для каждой строки v: грани всех 32 слоёв сразу разложены по строкам
            for (int v = 0; v < CHUNK; ++v) {
                for (int u = 0; u < CHUNK; ++u) {
                    const uint64_t b = self[ai][u][v], o = b | (block ? (*block)[ai][u][v] : 0);
                    const uint64_t f = sign == 0 ? (b & ~(o >> 1)) : (b & ~(o << 1));
                    t[u] = uint32_t(f >> 1);
                }
                transpose32(t);
                for (int d = 0; d < CHUNK; ++d) layerRows[d][v] = t[d];
            }

            for (int layer = 0; layer < CHUNK; ++layer) {
                uint32_t* rows = layerRows[layer];
                uint32_t any = 0;
                for (int v = 0; v < CHUNK; ++v) any |= rows[v];
                if (!any) continue;

                // строки - по материалам: материал читаем только у видимых граней
                int nmats = 0;
                for (int v = 0; v < CHUNK; ++v) {
                    for (uint32_t r = rows[v]; r; r &= r - 1) {
                        const int u = std::countr_zero(r);
                        int p[3];
                        p[ax.u] = u; p[ax.v] = v; p[ax.d] = layer;
                        const Material m = mat(p[0], p[1], p[2]);
                        int k = 0;
                        while (k < nmats && mats[k].mat != m) ++k;
                        if (k == nmats) { mats[k].mat = m; std::memset(mats[k].rows, 0, sizeof(mats[k].rows)); ++nmats; }
                        mats[k].rows[v] |= uint32_t(1) << u;
                    }
                }

                // слияние на битах: первая грань строки - ctz, ширина - длина отрезка единиц, вниз - пока строка
                // ниже содержит весь отрезок; забранное стираем из строк
                const int d = layer + (sign == 0 ? 1 : 0);  // плоскость: дальняя сторона вокселя для «+»
                for (int k = 0; k < nmats; ++k) {
                    uint32_t* r = mats[k].rows;
                    const Material m = mats[k].mat;
                    for (int v = 0; v < CHUNK; ++v) {
                        while (r[v]) {
                            const int u = std::countr_zero(r[v]);
                            const int w = std::countr_zero(~(r[v] >> u));      // единиц подряд, начиная с u
                            const uint32_t span = (w == 32 ? 0xFFFFFFFFu : ((uint32_t(1) << w) - 1)) << u;
                            int h = 1;
                            while (v + h < CHUNK && (r[v + h] & span) == span) { r[v + h] &= ~span; ++h; }
                            r[v] &= ~span;

                            int p0[3], p1[3], p2[3], p3[3];
                            p0[ax.d] = p1[ax.d] = p2[ax.d] = p3[ax.d] = d;
                            p0[ax.u] = u;     p0[ax.v] = v;
                            p1[ax.u] = u + w; p1[ax.v] = v;
                            p2[ax.u] = u + w; p2[ax.v] = v + h;
                            p3[ax.u] = u;     p3[ax.v] = v + h;
                            out.push_back(pack(p0[0], p0[1], p0[2], normal, m));
                            if (sign == 0) {
                                out.push_back(pack(p1[0], p1[1], p1[2], normal, m));
                                out.push_back(pack(p2[0], p2[1], p2[2], normal, m));
                                out.push_back(pack(p3[0], p3[1], p3[2], normal, m));
                            } else {
                                out.push_back(pack(p3[0], p3[1], p3[2], normal, m));
                                out.push_back(pack(p2[0], p2[1], p2[2], normal, m));
                                out.push_back(pack(p1[0], p1[1], p1[2], normal, m));
                            }
                            ++quads;
                        }
                    }
                }
            }
        }
    }
}

// Маски вдоль x и z - поворотом таблиц из вертикальных col[1] (внутренние 32 бита); кайма по x/z - из edge
// (столбцы с каймой 34×34). col[axis][u][v] - столбец вдоль оси d в точке (u, v) её плоскости.
void rotateMasks(uint64_t (&col)[3][CHUNK][CHUNK], const uint64_t (&edge)[CHUNK + 2][CHUNK + 2]) {
    uint64_t (&colX)[CHUNK][CHUNK] = col[0];   // [u = y][v = z], биты по x
    uint64_t (&colY)[CHUNK][CHUNK] = col[1];   // [u = z][v = x], биты по y
    uint64_t (&colZ)[CHUNK][CHUNK] = col[2];   // [u = x][v = y], биты по z
    uint32_t t[CHUNK];
    for (int z = 0; z < CHUNK; ++z) {                       // ось X: для каждого z таблица [x][биты y] -> [y][биты x]
        for (int x = 0; x < CHUNK; ++x) t[x] = uint32_t(colY[z][x] >> 1);
        transpose32(t);
        for (int y = 0; y < CHUNK; ++y) {
            uint64_t b = uint64_t(t[y]) << 1;
            b |= edge[z + 1][0] >> (y + 1) & 1;                                // x = -1
            b |= (edge[z + 1][CHUNK + 1] >> (y + 1) & 1) << 33;               // x = 32
            colX[y][z] = b;
        }
    }
    for (int x = 0; x < CHUNK; ++x) {                       // ось Z: для каждого x таблица [z][биты y] -> [y][биты z]
        for (int z = 0; z < CHUNK; ++z) t[z] = uint32_t(colY[z][x] >> 1);
        transpose32(t);
        for (int y = 0; y < CHUNK; ++y) {
            uint64_t b = uint64_t(t[y]) << 1;
            b |= edge[0][x + 1] >> (y + 1) & 1;                                // z = -1
            b |= (edge[CHUNK + 1][x + 1] >> (y + 1) & 1) << 33;               // z = 32
            colZ[x][y] = b;
        }
    }
}

// vox - воксели правленого куска; nullptr - нетронутый: маски и материалы формулой от высот и покрова
void meshImpl(ChunkPos pos, const Chunk* vox, const ColumnHeights& hm, const uint64_t (*air)[CHUNK + 2],
              std::vector<uint32_t>& out, MeshStats& stats) {
    out.clear();
    stats = {};
    const int32_t y0 = pos.y * CHUNK, x0w = pos.x * CHUNK, z0w = pos.z * CHUNK;
    // Столбцы с каймой по x и z (34×34), 34 бита по y, формулой: грунт - не выше дна, минус воздух пещер (Р26);
    // вода - выше дна и не выше уровня воды (Р28). Пещеры - только в грунте, под дном.
    // Р34: вода прозрачна - для граней она не «твёрдое»: грунт под водой даёт свои грани (видно дно), у воды
    // свои грани - только в воздух (поверхность, края водопадов); вода рисуется отдельным проходом.
    static thread_local uint64_t solid[CHUNK + 2][CHUNK + 2], wet[CHUNK + 2][CHUNK + 2];
    bool anyWater = false;
    for (int z = 0; z < CHUNK + 2; ++z)
        for (int x = 0; x < CHUNK + 2; ++x) {
            const uint64_t ground = columnFromHeight(hm.h[z][x], y0);
            solid[z][x] = ground & ~(air ? air[z][x] : 0);
            wet[z][x] = columnFromHeight(hm.water[z][x], y0) & ~ground;
            anyWater |= wet[z][x] != 0;
        }

    // ─── 1. Маски столбцов вдоль трёх осей: 34 бита (кайма + 32 + кайма) ─────────
    static thread_local uint64_t col[3][CHUNK][CHUNK], wcol[3][CHUNK][CHUNK];
    // вертикаль: формулой от высоты; правленый кусок - из его вокселей (кайма по высоте - по карте высот)
    constexpr uint64_t EDGE_Y = 1 | uint64_t(1) << 33;
    for (int z = 0; z < CHUNK; ++z) {
        for (int x = 0; x < CHUNK; ++x) {
            uint64_t b = solid[z + 1][x + 1], w = wet[z + 1][x + 1];
            if (vox) {
                b &= EDGE_Y;
                w &= EDGE_Y;
                for (int y = 0; y < CHUNK; ++y) {
                    const Material m = vox->at(x, y, z);
                    if (m == WATER) w |= uint64_t(1) << (y + 1);
                    else if (m != AIR) b |= uint64_t(1) << (y + 1);
                }
                anyWater |= w != 0;
            }
            col[1][z][x] = b;
            wcol[1][z][x] = w;
        }
    }
    rotateMasks(col, solid);

    // ─── 2-4. Грани грунта, затем воды: вода - в конце сетки (stats.waterQuads) ──
    emitFaces(col, nullptr, [&](int x, int y, int z) {
        return vox ? vox->at(x, y, z) : columnMaterial(hm.h[z + 1][x + 1], hm.cover[z][x], y0 + y, hm.water[z + 1][x + 1],
                                                       hm.rock[z][x], x0w + x, z0w + z);
    }, out, stats.quads);
    if (!anyWater) return;
    rotateMasks(wcol, wet);
    emitFaces(wcol, &col, [](int, int, int) { return Material(WATER); }, out, stats.waterQuads);
    stats.quads += stats.waterQuads;
}

}  // namespace

void buildMesh(const Chunk& c, const ColumnHeights& hm, std::vector<uint32_t>& out, MeshStats& stats) {
    meshImpl(c.pos, c.edited ? &c : nullptr, hm, nullptr, out, stats);
}

void buildMesh(ChunkPos pos, const ColumnHeights& hm, std::vector<uint32_t>& out, MeshStats& stats,
               const uint64_t (*air)[CHUNK + 2]) {
    meshImpl(pos, nullptr, hm, air, out, stats);
}

}  // namespace world
