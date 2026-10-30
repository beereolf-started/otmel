#include "world/terrain.h"

#include <algorithm>
#include <cmath>

#include "gen/hash.h"
#include "gen/noise.h"
#include "world/rivers.h"
#include "world/spread.h"

namespace world {
namespace {

// Зерно слоя: у каждого шума своё, перемешанное хешем - слои не повторяют друг друга.
// Равнины берут само зерно мира - их вид остался прежним.
enum Layer : int32_t { HILLS = 1, MOUNT, RELIEF, TEMP, RAIN, BELT, UPLIFT, CHAIN, CONTINENT, PLATEAU };
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
            gen::fbmSimplex2(x, z, p.upliftCellLog2, p.upliftOctaves, layerSeed(p.seed, UPLIFT)),
            gen::fbmSimplex2(x, z, p.chainCellLog2, p.chainOctaves, layerSeed(p.seed, CHAIN)),
            gen::fbmSimplex2(x, z, p.continentCellLog2, p.continentOctaves, layerSeed(p.seed, CONTINENT)),
            gen::fbmSimplex2(x, z, p.plateauCellLog2, p.plateauOctaves, layerSeed(p.seed, PLATEAU))};
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
                          fx::lerp(a.belt, b.belt, t), fx::lerp(a.uplift, b.uplift, t), fx::lerp(a.chain, b.chain, t),
                          fx::lerp(a.continent, b.continent, t), fx::lerp(a.plateau, b.plateau, t)};
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
    RiverSet rivers;
    rivers.build(p, x, z, x, z);
    return reliefAt(p, x, z, slowAt(p, x, z, minCellLog2), minCellLog2, &rivers);
}

Relief reliefAt(const TerrainParams& p, int32_t x, int32_t z, const SlowFields& slow, int minCellLog2,
                const RiverSet* rivers) {
    Relief r = baseReliefAt(p, x, z, slow, minCellLog2);
    // Р28: море - в океанических областях и на их берегах (доля суши < 1); на материке ниже уровня моря - сухо
    r.water = r.land < fx::ONE ? p.seaLevel - 1 : NO_WATER;
    // Р29: реки и озёра - прорезают долины, дают свою воду
    if (rivers) rivers->apply(p, x, z, r.height, r.water, r.shore);
    return r;
}

Relief baseReliefAt(const TerrainParams& p, int32_t x, int32_t z, const SlowFields& slow, int minCellLog2) {
    Relief r;
    // веса: плавные ступени по рельефности; горы забирают свою долю первыми, холмы - из остатка,
    // равнины - всё оставшееся. Сумма = 1 точно, по построению.
    const fx::q16 m = slow.relief;
    // Р25: суша или океан. В глубоком океане рельеф суши не считаем вовсе - только дно.
    r.land = fx::smoothstep(p.oceanFrom, p.oceanTo, slow.continent);
    const fx::q16 base = gen::fbm2(x, z, p.firstCellLog2, p.octaves, p.seed, minCellLog2);   // шум равнин - он же дна
    const int32_t floorH = p.seaLevel - p.oceanDepth + int32_t((int64_t(base) * (6 * VOXELS_PER_BLOCK)) >> fx::SHIFT);
    if (r.land == 0) {
        r.wPlains = fx::ONE;
        r.wHills = r.wMount = 0;
        r.height = floorH;
        return r;
    }
    fx::q16 core = 0;   // 0 у края цепи, 1 на её оси (вид 2: профиль массива)
    fx::q16 up = 0;     // высокогорье: 0..1
    fx::q16 gate = 0;   // присутствие цепи: 0..1 (Р24)
    if (p.mountainStyle == 0) {
        r.wMount = fx::mul(fx::smoothstep(p.mountFrom, p.mountTo, m), r.land);
        r.wHills = fx::mul(fx::mul(fx::smoothstep(p.hillsFrom, p.hillsTo, m), fx::ONE - r.wMount), r.land);
    } else {
        // Р23: горы - вдоль пояса цепи; по бокам - предгорья-холмы. Р24: цепь есть, где есть её «присутствие»
        // (свой медленный шум) - конец цепи растянут на километры, и рост цепи тоже умножается на присутствие
        // высокогорье: порог пояса ниже (цепь шире), рост - вдвое (ниже)
        up = fx::smoothstep(p.upliftFrom, p.upliftTo, slow.uplift);
        const fx::q16 from = p.chainFrom - fx::mul(up, p.upliftWiden), to = p.chainTo - fx::mul(up, p.upliftWiden);
        gate = fx::mul(std::max(fx::smoothstep(p.gateFrom, p.gateTo, slow.chain), up), r.land);   // в высокогорье цепь не рвётся; у моря - гаснет
        r.wMount = fx::mul(fx::smoothstep(from, to, slow.belt), gate);
        const fx::q16 foot = fx::mul(fx::smoothstep(from - p.footWidth, from, slow.belt), gate);
        r.wHills = fx::mul(fx::mul(std::max(fx::smoothstep(p.hillsFrom, p.hillsTo, m), foot), fx::ONE - r.wMount), r.land);
        core = fx::smoothstep(from, fx::ONE, slow.belt);
    }
    r.wPlains = fx::ONE - r.wMount - r.wHills;
    // высота - сумма видов с весами; вид с нулевым весом не считаем вовсе (в чистой зоне - одна формула)
    int64_t h = 0;
    if (r.wPlains > 0)
        h += int64_t(r.wPlains) * scaled(p.baseHeight, base, p.amplitude);
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
            const fx::q16 ridge =
                gen::ridged2(x, z, p.mountCellLog2, p.mountOctaves, layerSeed(p.seed, MOUNT), minCellLog2, false, p.mountGain);
            const fx::q16 ridgeShare = fx::ONE / 4 + fx::mul(core, fx::ONE * 3 / 4);
            // высокогорье - вдвое: и массив, и гребни, и колено; затухающая цепь - ниже (оседает, а не обрывается)
            const int64_t scale = fx::mul(fx::ONE + up, gate);
            const int32_t rise = int32_t((int64_t(core) * p.massifAmp) >> fx::SHIFT) +
                                 int32_t((int64_t(fx::mul(ridge, ridgeShare)) * p.ridgeAmp) >> fx::SHIFT);
            hm = p.mountBase + int32_t((rise * scale) >> fx::SHIFT);
            const int32_t knee = int32_t((int64_t(p.peakKnee) * scale) >> fx::SHIFT);
            if (hm > knee) hm = knee + (hm - knee) / 2;
        }
        h += int64_t(r.wMount) * hm;
    }
    // Р25: суша поднята над морем на landRise плюс плато (0..plateauRise); формулы видов рельефа отсчитаны от
    // прежней базы 32 блока - сдвигаем весь рельеф суши разом (и равнины, и холмы, и подножия гор)
    const int32_t ground = p.seaLevel + p.landRise - 32 * VOXELS_PER_BLOCK +
                           int32_t((int64_t(fx::smoothstep(p.plateauFrom, p.plateauTo, slow.plateau)) * p.plateauRise) >> fx::SHIFT);
    int32_t landH = static_cast<int32_t>(h >> fx::SHIFT) + ground;
    // Р49: суша не ниже моря + блок (мягкий максимум, переход 4 блока): у моря нет низины ниже его глади -
    // море не стоит стеной над сушей, уровень любой реки к устью (земля минус блок) не ниже моря
    {
        constexpr int32_t K = 4 * VOXELS_PER_BLOCK;
        const int32_t F = p.seaLevel + VOXELS_PER_BLOCK;
        const int32_t d = std::abs(landH - F), t = std::max(0, K - d);
        landH = std::max(landH, F) + int32_t(int64_t(t) * t / (4 * K));
    }
    // берег: суша плавно уходит на дно - то же смешивание весом, без ступеньки
    int32_t hh = floorH + int32_t((int64_t(r.land) * (landH - floorH)) >> fx::SHIFT);
    // мягкий потолок мира (Р10: до 1000 блоков): выше 960 каждый блок даёт четверть
    constexpr int32_t CEIL = 960 * VOXELS_PER_BLOCK;
    if (hh > CEIL) hh = CEIL + (hh - CEIL) / 4;
    r.height = hh;
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
    // Р25: дно и берег - песок (до 2 блоков над водой)
    if (height < p.seaLevel + 2 * VOXELS_PER_BLOCK) return {SAND, SAND};
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
    // Р29: звенья рек и озёра, задевающие кусок, - один раз на кусок
    static thread_local RiverSet rivers;
    rivers.build(p, x0, z0, x0 + CHUNK + 1, z0 + CHUNK + 1);
    static thread_local bool shore[CHUNK + 2][CHUNK + 2], genWet[CHUNK + 2][CHUNK + 2];
    out.minH = INT32_MAX;
    out.maxH = INT32_MIN;
    for (int z = 0; z < CHUNK + 2; ++z) {
        for (int x = 0; x < CHUNK + 2; ++x) {
            const Relief r = reliefAt(p, x0 + x, z0 + z, slow.at(x0 + x, z0 + z), 0, &rivers);
            out.h[z][x] = r.height;
            out.water[z][x] = r.water;
            shore[z][x] = r.shore;
            genWet[z][x] = r.water > r.height;
            const int32_t surface = std::max(r.height, r.water);   // Р28: поверхность - дно или вода над ним
            // Р37: нижний кусок - по дну: вода прозрачна, дно под ней должно строиться (было по воде - дыры в дне)
            out.minH = std::min(out.minH, r.height);
            out.maxH = std::max(out.maxH, surface);
            out.nearWater[z][x] = r.shore || r.water > r.height;
        }
    }
    // Р47: растекание как в Minecraft - по сетке блоков с запасом 10 блоков вокруг куска: источники -
    // сгенерированная вода, результат - сразу в уровень воды столбцов (вшит в генерацию)
    {
        constexpr int M = 10;
        const auto fdiv = [](int32_t v) { return v >= 0 ? v / VOXELS_PER_BLOCK : -((-v + VOXELS_PER_BLOCK - 1) / VOXELS_PER_BLOCK); };
        const int32_t bx0 = fdiv(x0) - M, bz0 = fdiv(z0) - M;
        const int n = std::max(fdiv(x0 + CHUNK + 1) - bx0, fdiv(z0 + CHUNK + 1) - bz0) + M + 1;
        static thread_local std::vector<int32_t> gb, sb, tb;
        static thread_local SlowGrid slowB;
        static thread_local RiverSet riversB;
        const int32_t rx0 = bx0 * VOXELS_PER_BLOCK, rz0 = bz0 * VOXELS_PER_BLOCK, rx1 = (bx0 + n) * VOXELS_PER_BLOCK, rz1 = (bz0 + n) * VOXELS_PER_BLOCK;
        slowB.build(p, rx0, rz0, rx1, rz1);
        riversB.build(p, rx0, rz0, rx1, rz1);
        gb.assign(size_t(n) * n, 0);
        sb.assign(size_t(n) * n, NO_WATER);
        for (int j = 0; j < n; ++j)
            for (int i = 0; i < n; ++i) {
                const int32_t x = (bx0 + i) * VOXELS_PER_BLOCK + VOXELS_PER_BLOCK / 2, z = (bz0 + j) * VOXELS_PER_BLOCK + VOXELS_PER_BLOCK / 2;
                const Relief r = reliefAt(p, x, z, slowB.at(x, z), 0, &riversB);
                gb[size_t(j) * n + i] = r.height;
                if (r.water > r.height) sb[size_t(j) * n + i] = r.water;
            }
        spreadWater(n, gb, sb, tb);
        for (int z = 0; z < CHUNK + 2; ++z)
            for (int x = 0; x < CHUNK + 2; ++x) {
                const int32_t t = tb[size_t(fdiv(z0 + z) - bz0) * n + size_t(fdiv(x0 + x) - bx0)];
                if (t == NO_WATER || t <= out.h[z][x] || t <= out.water[z][x] || genWet[z][x]) continue;   // источник не трогаем
                out.water[z][x] = t;
                out.maxH = std::max(out.maxH, t);
                out.nearWater[z][x] = true;
            }
        // Р48: гладь без отвесов. Где соседние мокрые блоки на разной высоте (ступень русла, падение со
        // уступа, ступени растекания), верх воды - билинейно по центрам мокрых блоков: ступень становится
        // скатом длиной в блок, отвесной грани воды нет. Затем растекание по столбцам вокселей: сухой сосед
        // мокрого столбца с дном ниже этой глади - тоже мокрый (у берега нет стенки воды над сухим дном).
        // гладь в центре блока: у мокрого - верх воды; у сухого рядом с водой - вровень с самой низкой соседней
        // водой или его дно, если оно ниже (гладь к нему опускается). Одно значение на центр - гладь без швов.
        static thread_local std::vector<int32_t> vb;
        vb.assign(size_t(n) * n, NO_WATER);
        for (int j = 1; j + 1 < n; ++j)
            for (int i = 1; i + 1 < n; ++i) {
                const size_t k = size_t(j) * n + i;
                if (tb[k] != NO_WATER) { vb[k] = tb[k]; continue; }
                int32_t low = INT32_MAX;
                for (int dj = -1; dj <= 1; ++dj)
                    for (int di = -1; di <= 1; ++di) {
                        const int32_t v = tb[size_t(j + dj) * n + size_t(i + di)];
                        if (v != NO_WATER) low = std::min(low, v);
                    }
                if (low != INT32_MAX) vb[k] = std::min(low, gb[k]);
            }
        static thread_local int32_t sm[CHUNK + 2][CHUNK + 2];
        for (int z = 0; z < CHUNK + 2; ++z)
            for (int x = 0; x < CHUNK + 2; ++x) {
                const int32_t qx = x0 + x - VOXELS_PER_BLOCK / 2, qz = z0 + z - VOXELS_PER_BLOCK / 2;
                const int32_t bi = fdiv(qx), bj = fdiv(qz);
                const float tx = (float(qx - bi * VOXELS_PER_BLOCK) + 0.5f) / VOXELS_PER_BLOCK;
                const float tz = (float(qz - bj * VOXELS_PER_BLOCK) + 0.5f) / VOXELS_PER_BLOCK;
                const size_t c = size_t(bj - bz0) * n + size_t(bi - bx0);
                const int32_t t[4] = {vb[c], vb[c + 1], vb[c + n], vb[c + n + 1]};
                const float w[4] = {(1 - tx) * (1 - tz), tx * (1 - tz), (1 - tx) * tz, tx * tz};
                sm[z][x] = NO_WATER;
                int32_t lo = INT32_MAX, hi = INT32_MIN;
                float st = 0;
                for (int k = 0; k < 4; ++k) {
                    if (t[k] == NO_WATER) break;
                    lo = std::min(lo, t[k]);
                    hi = std::max(hi, t[k]);
                    st += w[k] * float(t[k]);
                }
                if (lo == INT32_MAX || t[0] == NO_WATER || t[1] == NO_WATER || t[2] == NO_WATER || t[3] == NO_WATER) continue;
                const int32_t s = int32_t(std::lround(st)), h = out.h[z][x];
                sm[z][x] = s;
                // Р50: сгенерированная вода (реки - гладь уже без стенок, озёра, море) - как есть; сглаживается
                // только добавленная растеканием
                if (out.water[z][x] <= h || genWet[z][x]) continue;
                if (hi - lo < 2 && std::abs(out.water[z][x] - s) < 2) continue;   // ровная вода - как была
                out.water[z][x] = s > h ? s : h;
            }
        static thread_local int16_t queue[(CHUNK + 2) * (CHUNK + 2)];
        int qn = 0;
        for (int z = 0; z < CHUNK + 2; ++z)
            for (int x = 0; x < CHUNK + 2; ++x)
                if (out.water[z][x] > out.h[z][x]) queue[qn++] = int16_t(z * (CHUNK + 2) + x);
        for (int qi = 0; qi < qn; ++qi) {
            const int z = queue[qi] / (CHUNK + 2), x = queue[qi] % (CHUNK + 2);
            for (int d = 0; d < 4; ++d) {
                const int x2 = x + (d == 0) - (d == 1), z2 = z + (d == 2) - (d == 3);
                if (x2 < 0 || z2 < 0 || x2 >= CHUNK + 2 || z2 >= CHUNK + 2) continue;
                const int32_t h2 = out.h[z2][x2], s2 = sm[z2][x2];
                if (out.water[z2][x2] > h2 || s2 == NO_WATER || s2 <= h2 || h2 >= out.water[z][x] || s2 > out.water[z][x]) continue;
                out.water[z2][x2] = s2;
                queue[qn++] = int16_t(z2 * (CHUNK + 2) + x2);
            }
        }
        for (int z = 0; z < CHUNK + 2; ++z)
            for (int x = 0; x < CHUNK + 2; ++x)
                if (out.water[z][x] > out.h[z][x]) {
                    out.maxH = std::max(out.maxH, out.water[z][x]);
                    out.nearWater[z][x] = true;
                }
    }
    // климат и покров - только своим столбцам, не кайме; у воды рек и озёр - песок (Р29)
    for (int z = 0; z < CHUNK; ++z)
        for (int x = 0; x < CHUNK; ++x)
            out.cover[z][x] = shore[z + 1][x + 1]
                                  ? Cover{SAND, SAND}
                                  : coverAt(p, climateAt(p, x0 + 1 + x, z0 + 1 + z, slow.at(x0 + 1 + x, z0 + 1 + z)), out.h[z + 1][x + 1]);
}

void fillChunk(const ColumnHeights& heights, Chunk& c) {
    const int32_t y0 = c.pos.y * CHUNK;
    for (int z = 0; z < CHUNK; ++z)
        for (int x = 0; x < CHUNK; ++x)
            for (int y = 0; y < CHUNK; ++y)
                c.at(x, y, z) = columnMaterial(heights.h[z + 1][x + 1], heights.cover[z][x], y0 + y, heights.water[z + 1][x + 1]);
}

int32_t waterLevelAt(const TerrainParams&, const Relief& r) { return r.water; }

uint64_t checksum(const Chunk& c) {
    // FNV-1a по всем вокселям: любое расхождение хоть в одном вокселе меняет сумму
    uint64_t h = 1469598103934665603ull;
    for (Material m : c.voxels) { h ^= m; h *= 1099511628211ull; }
    return h;
}

}  // namespace world
