#include "world/caves.h"

#include <algorithm>

#include "gen/hash.h"
#include "gen/noise.h"

namespace world {
namespace {

constexpr int STEP_LOG2 = 3;                 // узлы через 8 вокселей (полблока)
constexpr int STEP = 1 << STEP_LOG2;
constexpr int MAX_NODES = (CHUNK + 2) / STEP + 2;   // узлов по стороне на кусок с каймой: 7

enum CaveLayer : int32_t { TUNNEL_A = 101, TUNNEL_B, HALL, HALL_FINE, GIANT, ENTRANCE };
inline uint32_t caveSeed(uint32_t seed, CaveLayer l) { return gen::hash2(l, 0xCA7E, seed); }

struct Node { fx::q16 tunnel, hall; };

Node caveNode(const TerrainParams& p, int32_t x, int32_t y, int32_t z) {
    const CaveParams& c = p.caves;
    Node n;
    // тоннели: два шума близки к нулю одновременно - труба; по высоте сжато вдвое - ходы пологие
    const int32_t ys = y * 2;
    const fx::q16 a = gen::simplex3(x, ys, z, c.tunnelCellLog2, caveSeed(p.seed, TUNNEL_A));
    const fx::q16 b = gen::simplex3(x, ys, z, c.tunnelCellLog2, caveSeed(p.seed, TUNNEL_B));
    n.tunnel = fx::mul(c.tunnelWidth, c.tunnelWidth) - fx::mul(a, a) - fx::mul(b, b);
    // залы: две октавы «сыра» выше порога
    const fx::q16 h = (2 * gen::simplex3(x, ys, z, c.hallCellLog2, caveSeed(p.seed, HALL)) +
                       gen::simplex3(x, ys, z, c.hallCellLog2 - 1, caveSeed(p.seed, HALL_FINE))) / 3;
    n.hall = h - c.hallFrom;
    // гигантские полости: ниже giantTop, в полную силу - ниже giantFull
    if (y < c.giantTop) {
        const fx::q16 fade = fx::smoothstep(-c.giantTop, -c.giantFull, -y);
        const fx::q16 g = gen::simplex3(x, ys, z, c.giantCellLog2, caveSeed(p.seed, GIANT));
        n.hall = std::max(n.hall, g - c.giantFrom - (fx::ONE - fade));
    }
    return n;
}

}  // namespace

bool caveMouth(const TerrainParams& p, int32_t x, int32_t z, int32_t top, int32_t widenBlocks) {
    const CaveParams& c = p.caves;
    if (gen::simplex2(x, z, c.entranceCellLog2, caveSeed(p.seed, ENTRANCE)) <= c.entranceFrom) return false;
    const fx::q16 w2 = fx::mul(c.tunnelWidth, c.tunnelWidth);
    const int64_t k = std::min<int64_t>(15, int64_t(widenBlocks) * widenBlocks / 4);   // толщина ×√(1+k)
    const Node n = caveNode(p, x, top - c.tunnelCover / 2, z);
    return int64_t(n.tunnel) + w2 * k > 0;
}

bool caveEntranceNear(const TerrainParams& p, int32_t cx, int32_t cz) {
    const CaveParams& c = p.caves;
    return gen::simplex2(cx * CHUNK - 1 + CHUNK / 2, cz * CHUNK - 1 + CHUNK / 2, c.entranceCellLog2, caveSeed(p.seed, ENTRANCE)) >=
           c.entranceFrom - fx::ONE / 10;
}

bool caveAir(const TerrainParams& p, ChunkPos pos, const ColumnHeights& hm, uint64_t air[CHUNK + 2][CHUNK + 2]) {
    const CaveParams& c = p.caves;
    const int32_t x0 = pos.x * CHUNK - 1, y0 = pos.y * CHUNK - 1, z0 = pos.z * CHUNK - 1;   // начало каймы

    // Быстрый отказ 1: кусок целиком в верхнем слое (ближе tunnelCover к поверхности) и над ним нет входа -
    // тоннели сюда не выходят; залы - тем более (они глубже).
    int32_t maxDepth = INT32_MIN;
    for (int z = 0; z < CHUNK + 2; ++z)
        for (int x = 0; x < CHUNK + 2; ++x) maxDepth = std::max(maxDepth, hm.h[z][x] - y0);
    if (maxDepth < 0) return false;   // весь кусок над поверхностью
    const auto entranceAt = [&](int32_t x, int32_t z) {
        return gen::simplex2(x, z, c.entranceCellLog2, caveSeed(p.seed, ENTRANCE)) > c.entranceFrom;
    };
    if (maxDepth < c.tunnelCover &&
        gen::simplex2(x0 + CHUNK / 2, z0 + CHUNK / 2, c.entranceCellLog2, caveSeed(p.seed, ENTRANCE)) < c.entranceFrom - fx::ONE / 10)
        return false;

    // узлы каналов
    const int32_t nx0 = x0 >> STEP_LOG2, ny0 = y0 >> STEP_LOG2, nz0 = z0 >> STEP_LOG2;
    const int nx = ((x0 + CHUNK + 1) >> STEP_LOG2) - nx0 + 2;
    const int ny = ((y0 + CHUNK + 1) >> STEP_LOG2) - ny0 + 2;
    const int nz = ((z0 + CHUNK + 1) >> STEP_LOG2) - nz0 + 2;
    static thread_local Node nodes[MAX_NODES][MAX_NODES][MAX_NODES];
    fx::q16 maxT = INT32_MIN, maxH = INT32_MIN;
    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                const Node n = caveNode(p, (nx0 + i) << STEP_LOG2, (ny0 + j) << STEP_LOG2, (nz0 + k) << STEP_LOG2);
                nodes[k][j][i] = n;
                maxT = std::max(maxT, n.tunnel);
                maxH = std::max(maxH, n.hall);
            }
    // Быстрый отказ 2: смешивание не выходит за пределы значений в узлах
    if (maxT <= 0 && maxH <= 0) return false;

    // по вокселям - только ниже поверхности своего столбца (выше и так воздух)
    constexpr int32_t mask = STEP - 1;
    constexpr int FR = fx::SHIFT - STEP_LOG2;
    bool any = false;
    for (int z = 0; z < CHUNK + 2; ++z)
        for (int x = 0; x < CHUNK + 2; ++x) {
            const int32_t wx = x0 + x, wz = z0 + z, top = hm.h[z][x];
            const int ix = (wx >> STEP_LOG2) - nx0, iz = (wz >> STEP_LOG2) - nz0;
            const fx::q16 fx_ = (wx & mask) << FR, fz = (wz & mask) << FR;
            int entrance = -1;   // считаем вход, только если понадобился
            uint64_t bits = 0;
            for (int yi = 0; yi < CHUNK + 2; ++yi) {
                const int32_t wy = y0 + yi;
                if (wy > top) break;
                const int32_t d = top - wy;   // глубина под поверхностью, воксели
                const int iy = (wy >> STEP_LOG2) - ny0;
                const fx::q16 fy = (wy & mask) << FR;
                // смешивание по трём осям: x, затем z, затем y
                const auto tri = [&](auto get) {
                    const fx::q16 a = fx::lerp(get(nodes[iz][iy][ix]), get(nodes[iz][iy][ix + 1]), fx_);
                    const fx::q16 b = fx::lerp(get(nodes[iz + 1][iy][ix]), get(nodes[iz + 1][iy][ix + 1]), fx_);
                    const fx::q16 e = fx::lerp(get(nodes[iz][iy + 1][ix]), get(nodes[iz][iy + 1][ix + 1]), fx_);
                    const fx::q16 f = fx::lerp(get(nodes[iz + 1][iy + 1][ix]), get(nodes[iz + 1][iy + 1][ix + 1]), fx_);
                    return fx::lerp(fx::lerp(a, b, fz), fx::lerp(e, f, fz), fy);
                };
                bool open = false;
                if (maxT > 0) {
                    fx::q16 t = tri([](const Node& n) { return n.tunnel; });
                    if (t > 0 && d < c.tunnelCover) {
                        // под водой и на берегу входов нет: вода прозрачна (Р34) - дыра в дне или в боку русла
                        // видна насквозь (Р37: и у берега - вход в берегу открывался в бок русла)
                        if (entrance < 0) entrance = !hm.nearWater[z][x] && entranceAt(wx, wz) ? 1 : 0;
                        // не у входа: тоннель гаснет за последний блок перед верхним слоем
                        if (!entrance) t -= fx::q16(int64_t(fx::mul(c.tunnelWidth, c.tunnelWidth)) * (c.tunnelCover - d) / VOXELS_PER_BLOCK);
                    }
                    open = t > 0;
                }
                if (!open && maxH > 0 && d >= c.hallDepth - 8 * VOXELS_PER_BLOCK) {
                    fx::q16 h = tri([](const Node& n) { return n.hall; });
                    // залы гаснут к верху на 8 блоках над hallDepth - потолок не плоский
                    if (d < c.hallDepth) h -= fx::q16(int64_t(fx::ONE * 3 / 10) * (c.hallDepth - d) / (8 * VOXELS_PER_BLOCK));
                    open = h > 0;
                }
                if (open) bits |= uint64_t(1) << yi;
            }
            air[z][x] = bits;
            any |= bits != 0;
        }
    return any;
}

}  // namespace world
