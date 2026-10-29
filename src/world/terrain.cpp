#include "world/terrain.h"

#include <algorithm>

#include "gen/hash.h"
#include "gen/noise.h"

namespace world {
namespace {

// Зерно слоя: у каждого шума своё, перемешанное хешем - слои не повторяют друг друга.
// Равнины берут само зерно мира - их вид остался прежним.
enum Layer : int32_t { HILLS = 1, MOUNT, RELIEF, TEMP, RAIN, BELT, UPLIFT };
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

// медленные поля в узле сетки
SlowFields slowNode(const TerrainParams& p, int32_t nx, int32_t nz, int minCellLog2) {
    const int32_t x = nx << SLOW_STEP_LOG2, z = nz << SLOW_STEP_LOG2;
    // пояс цепи: 1 - |s| со сглаженным модулем у нуля (|s| < e -> (s² + e²) / 2e): без излома по оси цепи
    const fx::q16 s = gen::fbmSimplex2(x, z, p.beltCellLog2, p.beltOctaves, layerSeed(p.seed, BELT));
    constexpr fx::q16 e = fx::ONE / 20;
    const fx::q16 a = s < 0 ? -s : s;
    const fx::q16 soft = a < e ? fx::q16((int64_t(s) * s + int64_t(e) * e) / (2 * int64_t(e))) : a;
    return {gen::fbmValue2(x, z, p.reliefCellLog2, p.reliefOctaves, layerSeed(p.seed, RELIEF), minCellLog2),
            gen::fbmValue2(x, z, p.climateCellLog2, p.climateOctaves, layerSeed(p.seed, TEMP)),
            gen::fbmValue2(x, z, p.climateCellLog2, p.climateOctaves, layerSeed(p.seed, RAIN)), fx::ONE - soft,
            gen::fbmSimplex2(x, z, p.upliftCellLog2, p.upliftOctaves, layerSeed(p.seed, UPLIFT))};
}

// Линейное смешивание четырёх узлов. Узел с нулевым весом не нужен (точка на линии сетки) - его и не
// спрашиваем: lerp(a, b, 0) = a точно, поэтому результат тот же до бита.
template <class Node>
SlowFields bilerp(int32_t x, int32_t z, Node node) {
    constexpr int32_t mask = (1 << SLOW_STEP_LOG2) - 1;
    const int32_t nx = x >> SLOW_STEP_LOG2, nz = z >> SLOW_STEP_LOG2;
    const fx::q16 fx_ = (x & mask) << (fx::SHIFT - SLOW_STEP_LOG2), fz = (z & mask) << (fx::SHIFT - SLOW_STEP_LOG2);
    const auto mix = [](const SlowFields& a, const SlowFields& b, fx::q16 t) {
        return SlowFields{fx::lerp(a.relief, b.relief, t), fx::lerp(a.temp, b.temp, t), fx::lerp(a.rain, b.rain, t),
                          fx::lerp(a.belt, b.belt, t), fx::lerp(a.uplift, b.uplift, t)};
    };
    SlowFields top = node(nx, nz);
    if (fx_) top = mix(top, node(nx + 1, nz), fx_);
    if (!fz) return top;
    SlowFields bot = node(nx, nz + 1);
    if (fx_) bot = mix(bot, node(nx + 1, nz + 1), fx_);
    return mix(top, bot, fz);
}

}  // namespace

SlowFields slowAt(const TerrainParams& p, int32_t x, int32_t z, int minCellLog2) {
    return bilerp(x, z, [&](int32_t nx, int32_t nz) { return slowNode(p, nx, nz, minCellLog2); });
}

void SlowGrid::build(const TerrainParams& p, int32_t x0, int32_t z0, int32_t x1, int32_t z1, int minCellLog2) {
    nx0 = x0 >> SLOW_STEP_LOG2;
    nz0 = z0 >> SLOW_STEP_LOG2;
    w = (x1 >> SLOW_STEP_LOG2) - nx0 + 2;
    h = (z1 >> SLOW_STEP_LOG2) - nz0 + 2;
    nodes.resize(size_t(w) * h);
    for (int j = 0; j < h; ++j)
        for (int i = 0; i < w; ++i) nodes[size_t(j) * w + i] = slowNode(p, nx0 + i, nz0 + j, minCellLog2);
}

SlowFields SlowGrid::at(int32_t x, int32_t z) const {
    return bilerp(x, z, [&](int32_t nx, int32_t nz) { return nodes[size_t(nz - nz0) * w + size_t(nx - nx0)]; });
}

Relief reliefAt(const TerrainParams& p, int32_t x, int32_t z, int minCellLog2) {
    return reliefAt(p, x, z, slowAt(p, x, z, minCellLog2), minCellLog2);
}

Relief reliefAt(const TerrainParams& p, int32_t x, int32_t z, const SlowFields& slow, int minCellLog2) {
    Relief r;
    // веса: плавные ступени по рельефности; горы забирают свою долю первыми, холмы - из остатка,
    // равнины - всё оставшееся. Сумма = 1 точно, по построению.
    const fx::q16 m = slow.relief;
    fx::q16 core = 0;   // 0 у края цепи, 1 на её оси (вид 2: профиль массива)
    fx::q16 up = 0;     // высокогорье: 0..1
    if (p.mountainStyle == 0) {
        r.wMount = fx::smoothstep(p.mountFrom, p.mountTo, m);
        r.wHills = fx::mul(fx::smoothstep(p.hillsFrom, p.hillsTo, m), fx::ONE - r.wMount);
    } else {
        // Р23: горы - вдоль пояса цепи; цепь прерывается, где рельефность низка; по бокам - предгорья-холмы
        // высокогорье: порог пояса ниже (цепь шире), рост - вдвое (ниже)
        up = fx::smoothstep(p.upliftFrom, p.upliftTo, slow.uplift);
        const fx::q16 from = p.chainFrom - fx::mul(up, p.upliftWiden), to = p.chainTo - fx::mul(up, p.upliftWiden);
        const fx::q16 gate = std::max(fx::smoothstep(p.gateFrom, p.gateTo, m), up);   // в высокогорье цепь не рвётся
        r.wMount = fx::mul(fx::smoothstep(from, to, slow.belt), gate);
        const fx::q16 foot = fx::mul(fx::smoothstep(from - p.footWidth, from, slow.belt), gate);
        r.wHills = fx::mul(std::max(fx::smoothstep(p.hillsFrom, p.hillsTo, m), foot), fx::ONE - r.wMount);
        core = fx::smoothstep(from, fx::ONE, slow.belt);
    }
    r.wPlains = fx::ONE - r.wMount - r.wHills;
    // высота - сумма видов с весами; вид с нулевым весом не считаем вовсе (в чистой зоне - одна формула)
    int64_t h = 0;
    if (r.wPlains > 0)
        h += int64_t(r.wPlains) * scaled(p.baseHeight, gen::fbm2(x, z, p.firstCellLog2, p.octaves, p.seed, minCellLog2), p.amplitude);
    if (r.wHills > 0)
        h += int64_t(r.wHills) *
             scaled(p.hillsBase, gen::fbmSimplex2(x, z, p.hillsCellLog2, p.hillsOctaves, layerSeed(p.seed, HILLS), minCellLog2), p.hillsAmp);
    if (r.wMount > 0) {
        int32_t hm;
        if (p.mountainStyle < 2) {
            hm = scaled(p.mountBase, gen::ridged2(x, z, p.mountCellLog2, p.mountOctaves, layerSeed(p.seed, MOUNT), minCellLog2), p.mountAmp);
        } else {
            // массив: гладкий подъём к оси цепи; гребни без квадрата, сильнее у оси (25% у края, 100% на оси);
            // «колено»: выше peakKnee каждый воксель даёт половину - иглы оседают в плечи и купола
            const fx::q16 ridge = gen::ridged2(x, z, p.mountCellLog2, p.mountOctaves, layerSeed(p.seed, MOUNT), minCellLog2, false);
            const fx::q16 ridgeShare = fx::ONE / 4 + fx::mul(core, fx::ONE * 3 / 4);
            // высокогорье - вдвое: и массив, и гребни, и колено
            const int64_t scale = fx::ONE + up;
            const int32_t rise = int32_t((int64_t(core) * p.massifAmp) >> fx::SHIFT) +
                                 int32_t((int64_t(fx::mul(ridge, ridgeShare)) * p.ridgeAmp) >> fx::SHIFT);
            hm = p.mountBase + int32_t((rise * scale) >> fx::SHIFT);
            const int32_t knee = int32_t((int64_t(p.peakKnee) * scale) >> fx::SHIFT);
            if (hm > knee) hm = knee + (hm - knee) / 2;
        }
        h += int64_t(r.wMount) * hm;
    }
    r.height = static_cast<int32_t>(h >> fx::SHIFT);
    return r;
}

Climate climateAt(const TerrainParams& p, int32_t x, int32_t z) { return climateAt(p, x, z, slowAt(p, x, z)); }

Climate climateAt(const TerrainParams& p, int32_t x, int32_t z, const SlowFields& slow) {
    Climate c;
    // температура: пояса по z (к +z жарче, к -z холоднее - север, как в TFC) плюс местный шум
    c.temp = p.tempMid + fx::mul(p.tempSwing, triangle(z, p.climateBand)) + fx::mul(p.tempNoise, slow.temp);
    // осадки: пояса по x плюс шум, в пределах 0..2·rainMid
    const fx::q16 rn = slow.rain;
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
    // медленные поля - по узлам сетки: на столбец с каймой (34×34 вокселя) - 2-3 узла по стороне вместо 1156
    static thread_local SlowGrid slow;
    slow.build(p, x0, z0, x0 + CHUNK + 1, z0 + CHUNK + 1);
    out.minH = INT32_MAX;
    out.maxH = INT32_MIN;
    for (int z = 0; z < CHUNK + 2; ++z) {
        for (int x = 0; x < CHUNK + 2; ++x) {
            const int32_t h = reliefAt(p, x0 + x, z0 + z, slow.at(x0 + x, z0 + z)).height;
            out.h[z][x] = h;
            out.minH = std::min(out.minH, h);
            out.maxH = std::max(out.maxH, h);
        }
    }
    // климат и покров - только своим столбцам, не кайме
    for (int z = 0; z < CHUNK; ++z)
        for (int x = 0; x < CHUNK; ++x)
            out.cover[z][x] = coverAt(p, climateAt(p, x0 + 1 + x, z0 + 1 + z, slow.at(x0 + 1 + x, z0 + 1 + z)), out.h[z + 1][x + 1]);
}

void fillChunk(const ColumnHeights& heights, Chunk& c) {
    const int32_t y0 = c.pos.y * CHUNK;
    for (int z = 0; z < CHUNK; ++z)
        for (int x = 0; x < CHUNK; ++x)
            for (int y = 0; y < CHUNK; ++y)
                c.at(x, y, z) = columnMaterial(heights.h[z + 1][x + 1], heights.cover[z][x], y0 + y);
}

uint64_t checksum(const Chunk& c) {
    // FNV-1a по всем вокселям: любое расхождение хоть в одном вокселе меняет сумму
    uint64_t h = 1469598103934665603ull;
    for (Material m : c.voxels) { h ^= m; h *= 1099511628211ull; }
    return h;
}

}  // namespace world
