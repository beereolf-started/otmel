// ЭТАЛОН: прежняя сборка сетки перебором вокселей (до оптимизации Р18). Нужна только тестам - новая сборка
// обязана давать тот же набор прямоугольников.
#include "world/mesher.h"

#include <cstring>

namespace world {
namespace {

// Твёрдый ли воксель (x, y, z) в координатах куска, с каймой [-1..32]. Внутри куска - по вокселям,
// снаружи - по карте высот: у рельефа всё, что не выше поверхности, твёрдое.
inline bool solidAt(const Chunk& c, const ColumnHeights& hm, int x, int y, int z) {
    if (x >= 0 && x < CHUNK && y >= 0 && y < CHUNK && z >= 0 && z < CHUNK) return c.at(x, y, z) != AIR;
    return c.pos.y * CHUNK + y <= hm.h[z + 1][x + 1];
}

// Оси граней. Для каждой оси d плоскость грани задаётся осями (u, v) так, что u × v = +d (правая тройка) -
// тогда обход p0 → p0+u·w → p0+u·w+v·h → p0+v·h идёт против часовой стрелки, если смотреть снаружи.
//   X: u = y, v = z      Y: u = z, v = x      Z: u = x, v = y
struct Axis { int d, u, v; };
constexpr Axis AXES[3] = {{0, 1, 2}, {1, 2, 0}, {2, 0, 1}};

inline uint32_t pack(int x, int y, int z, int normal, Material m) {
    return uint32_t(x) | uint32_t(y) << 6 | uint32_t(z) << 12 | uint32_t(normal) << 18 | uint32_t(m) << 21;
}

}  // namespace

void buildMeshReference(const Chunk& c, const ColumnHeights& hm, std::vector<uint32_t>& out, MeshStats& stats) {
    out.clear();
    stats = {};
    // столбец вдоль оси d: 34 бита (кайма + 32 + кайма), бит i - воксель с координатой i-1 по оси d
    static thread_local uint64_t col[CHUNK][CHUNK];
    static thread_local Material plane[CHUNK][CHUNK];
    static thread_local bool used[CHUNK][CHUNK];

    for (int ai = 0; ai < 3; ++ai) {
        const Axis ax = AXES[ai];
        // 1. Маски столбцов: для каждой точки плоскости (u, v) - 34 бита твёрдости вдоль d
        for (int u = 0; u < CHUNK; ++u) {
            for (int v = 0; v < CHUNK; ++v) {
                uint64_t bits = 0;
                int p[3];
                p[ax.u] = u;
                p[ax.v] = v;
                for (int i = 0; i < CHUNK + 2; ++i) {
                    p[ax.d] = i - 1;
                    if (solidAt(c, hm, p[0], p[1], p[2])) bits |= uint64_t(1) << i;
                }
                col[u][v] = bits;
            }
        }
        // 2. Два направления вдоль оси: грань видна, где воксель твёрдый, а сосед в сторону грани - воздух.
        //    Для всех 32 вокселей столбца разом: «+»: col & ~(col >> 1)   «−»: col & ~(col << 1)
        for (int sign = 0; sign < 2; ++sign) {
            const int normal = ai * 2 + sign;          // 0 +X, 1 -X, 2 +Y, 3 -Y, 4 +Z, 5 -Z
            for (int layer = 0; layer < CHUNK; ++layer) {
                // 3. Плоскость слоя: материал там, где есть грань, иначе 0
                bool any = false;
                for (int u = 0; u < CHUNK; ++u) {
                    for (int v = 0; v < CHUNK; ++v) {
                        const uint64_t b = col[u][v];
                        const uint64_t faces = sign == 0 ? (b & ~(b >> 1)) : (b & ~(b << 1));
                        Material m = AIR;
                        if (faces >> (layer + 1) & 1) {
                            int p[3];
                            p[ax.u] = u; p[ax.v] = v; p[ax.d] = layer;
                            m = c.at(p[0], p[1], p[2]);
                            any = true;
                        }
                        plane[u][v] = m;
                    }
                }
                if (!any) continue;
                // 4. Слияние: от каждой несобранной грани тянем прямоугольник вдоль u, потом вдоль v,
                //    пока материал тот же. Один прямоугольник вместо десятков одинаковых квадратиков.
                std::memset(used, 0, sizeof(used));
                for (int v = 0; v < CHUNK; ++v) {
                    for (int u = 0; u < CHUNK; ++u) {
                        const Material m = plane[u][v];
                        if (m == AIR || used[u][v]) continue;
                        int w = 1;
                        while (u + w < CHUNK && plane[u + w][v] == m && !used[u + w][v]) ++w;
                        int h = 1;
                        for (; v + h < CHUNK; ++h) {
                            bool rowOk = true;
                            for (int k = 0; k < w && rowOk; ++k) rowOk = plane[u + k][v + h] == m && !used[u + k][v + h];
                            if (!rowOk) break;
                        }
                        for (int dv = 0; dv < h; ++dv)
                            for (int du = 0; du < w; ++du) used[u + du][v + dv] = true;

                        // углы: плоскость грани - на дальней стороне вокселя для «+», на ближней для «−»
                        const int d = layer + (sign == 0 ? 1 : 0);
                        int p0[3], p1[3], p2[3], p3[3];
                        p0[ax.d] = p1[ax.d] = p2[ax.d] = p3[ax.d] = d;
                        p0[ax.u] = u;     p0[ax.v] = v;
                        p1[ax.u] = u + w; p1[ax.v] = v;
                        p2[ax.u] = u + w; p2[ax.v] = v + h;
                        p3[ax.u] = u;     p3[ax.v] = v + h;
                        if (sign == 0) {   // против часовой снаружи
                            out.push_back(pack(p0[0], p0[1], p0[2], normal, m));
                            out.push_back(pack(p1[0], p1[1], p1[2], normal, m));
                            out.push_back(pack(p2[0], p2[1], p2[2], normal, m));
                            out.push_back(pack(p3[0], p3[1], p3[2], normal, m));
                        } else {           // обратная сторона - обратный обход
                            out.push_back(pack(p0[0], p0[1], p0[2], normal, m));
                            out.push_back(pack(p3[0], p3[1], p3[2], normal, m));
                            out.push_back(pack(p2[0], p2[1], p2[2], normal, m));
                            out.push_back(pack(p1[0], p1[1], p1[2], normal, m));
                        }
                        ++stats.quads;
                    }
                }
            }
        }
    }
}

}  // namespace world
