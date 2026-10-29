#include "world/terrain.h"

#include <algorithm>

#include "gen/hash.h"
#include "gen/noise.h"

namespace world {
namespace {

// Зерно слоя: у каждого шума своё, перемешанное хешем - слои не повторяют друг друга.
// Равнины берут само зерно мира - их вид остался прежним.
enum Layer : int32_t { HILLS = 1, MOUNT, RELIEF, TEMP, RAIN };
inline uint32_t layerSeed(uint32_t seed, Layer l) { return gen::hash2(l, 0x51ED, seed); }

// Треугольная волна периода 2·band: 0 в нуле, +1 на band/2, -1 на -band/2. Пояса климата - линейные,
// как в TFC; ни тригонометрии, ни плавающей точки.
fx::q16 triangle(int32_t v, int32_t band) {
    const int64_t period = int64_t(band) * 2;
    int64_t ph = (int64_t(v) + band / 2) % period;
    if (ph < 0) ph += period;
    const int64_t d = ph < band ? ph : period - ph;   // 0..band
    return static_cast<fx::q16>(((2 * d - band) << fx::SHIFT) / band);
}

inline int32_t scaled(int32_t base, fx::q16 n, int32_t amp) {
    return base + static_cast<int32_t>((static_cast<int64_t>(n) * amp) >> fx::SHIFT);
}

}  // namespace

Relief reliefAt(const TerrainParams& p, int32_t x, int32_t z) {
    Relief r;
    // веса: плавные ступени по рельефности; горы забирают свою долю первыми, холмы - из остатка,
    // равнины - всё оставшееся. Сумма = 1 точно, по построению.
    const fx::q16 m = gen::fbmValue2(x, z, p.reliefCellLog2, p.reliefOctaves, layerSeed(p.seed, RELIEF));
    r.wMount = fx::smoothstep(p.mountFrom, p.mountTo, m);
    r.wHills = fx::mul(fx::smoothstep(p.hillsFrom, p.hillsTo, m), fx::ONE - r.wMount);
    r.wPlains = fx::ONE - r.wMount - r.wHills;
    // высота - сумма видов с весами; вид с нулевым весом не считаем вовсе (в чистой зоне - одна формула)
    int64_t h = 0;
    if (r.wPlains > 0)
        h += int64_t(r.wPlains) * scaled(p.baseHeight, gen::fbm2(x, z, p.firstCellLog2, p.octaves, p.seed), p.amplitude);
    if (r.wHills > 0)
        h += int64_t(r.wHills) *
             scaled(p.hillsBase, gen::fbmSimplex2(x, z, p.hillsCellLog2, p.hillsOctaves, layerSeed(p.seed, HILLS)), p.hillsAmp);
    if (r.wMount > 0)
        h += int64_t(r.wMount) *
             scaled(p.mountBase, gen::ridged2(x, z, p.mountCellLog2, p.mountOctaves, layerSeed(p.seed, MOUNT)), p.mountAmp);
    r.height = static_cast<int32_t>(h >> fx::SHIFT);
    return r;
}

Climate climateAt(const TerrainParams& p, int32_t x, int32_t z) {
    Climate c;
    // температура: пояса по z (к +z жарче, к -z холоднее - север, как в TFC) плюс местный шум
    const fx::q16 tn = gen::fbmValue2(x, z, p.climateCellLog2, p.climateOctaves, layerSeed(p.seed, TEMP));
    c.temp = p.tempMid + fx::mul(p.tempSwing, triangle(z, p.climateBand)) + fx::mul(p.tempNoise, tn);
    // осадки: пояса по x плюс шум, в пределах 0..2·rainMid
    const fx::q16 rn = gen::fbmValue2(x, z, p.climateCellLog2, p.climateOctaves, layerSeed(p.seed, RAIN));
    c.rain = std::clamp(p.rainMid + fx::mul(p.rainSwing, triangle(x, p.climateBand)) + fx::mul(p.rainNoise, rn), 0,
                        2 * p.rainMid);
    return c;
}

fx::q16 temperatureAt(const TerrainParams& p, const Climate& c, int32_t height) {
    // выше уровня моря - холоднее; ниже не теплеет (впадины и долины - как на уровне моря)
    const int32_t above = std::max(0, height - p.seaLevel);
    return c.temp - static_cast<fx::q16>(int64_t(above) * p.lapsePerVoxel);
}

Cover coverAt(const TerrainParams& p, const Climate& c, int32_t height) {
    const fx::q16 t = temperatureAt(p, c, height);
    if (t < p.snowBelow) return {SNOW, DIRT};
    if (c.rain < p.desertRain && t > p.desertTemp) return {SAND, SAND};
    return {GRASS, DIRT};
}

void columnHeights(const TerrainParams& p, int32_t chunkX, int32_t chunkZ, ColumnHeights& out) {
    const int32_t x0 = chunkX * CHUNK - 1, z0 = chunkZ * CHUNK - 1;
    out.minH = INT32_MAX;
    out.maxH = INT32_MIN;
    for (int z = 0; z < CHUNK + 2; ++z) {
        for (int x = 0; x < CHUNK + 2; ++x) {
            const int32_t h = reliefAt(p, x0 + x, z0 + z).height;
            out.h[z][x] = h;
            out.minH = std::min(out.minH, h);
            out.maxH = std::max(out.maxH, h);
        }
    }
    // климат и покров - только своим столбцам, не кайме
    for (int z = 0; z < CHUNK; ++z)
        for (int x = 0; x < CHUNK; ++x)
            out.cover[z][x] = coverAt(p, climateAt(p, x0 + 1 + x, z0 + 1 + z), out.h[z + 1][x + 1]);
}

void fillChunk(const ColumnHeights& heights, Chunk& c) {
    const int32_t y0 = c.pos.y * CHUNK;
    constexpr int32_t UNDER_DEPTH = 3 * VOXELS_PER_BLOCK;
    for (int z = 0; z < CHUNK; ++z) {
        for (int x = 0; x < CHUNK; ++x) {
            const int32_t top = heights.h[z + 1][x + 1];
            const Cover cv = heights.cover[z][x];
            for (int y = 0; y < CHUNK; ++y) {
                const int32_t wy = y0 + y;
                Material m = AIR;
                if (wy == top) m = cv.top;
                else if (wy < top && wy >= top - UNDER_DEPTH) m = cv.under;
                else if (wy < top) m = STONE;
                c.at(x, y, z) = m;
            }
        }
    }
}

uint64_t checksum(const Chunk& c) {
    // FNV-1a по всем вокселям: любое расхождение хоть в одном вокселе меняет сумму
    uint64_t h = 1469598103934665603ull;
    for (Material m : c.voxels) { h ^= m; h *= 1099511628211ull; }
    return h;
}

}  // namespace world
