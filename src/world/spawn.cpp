#include "world/spawn.h"

#include <algorithm>
#include <cmath>

#include "world/caves.h"

namespace world {
namespace {

int32_t floorDiv(int32_t a, int32_t b) { return a >= 0 ? a / b : -((-a + b - 1) / b); }

// Настоящий верх столбца в вокселях: как строится мир, то есть с каймой куска и со срезом земли у
// выхода шахты (Р79). Отдельно - уровень воды, если она здесь есть. Проба дорогая: считается весь
// кусок 34×34, поэтому ею проверяется только то, что уже прошло грубый отбор по рельефу.
void surfaceAt(const TerrainParams& p, int32_t vx, int32_t vz, int32_t& top, int32_t& water) {
    static thread_local ColumnHeights hm;
    static thread_local uint8_t cut[CHUNK + 2][CHUNK + 2];
    const int32_t cx = floorDiv(vx, CHUNK), cz = floorDiv(vz, CHUNK);
    columnHeights(p, cx, cz, hm);
    soilCut(p, cx, cz, hm, cut);
    const int lx = vx - cx * CHUNK, lz = vz - cz * CHUNK;
    top = hm.h[lz + 1][lx + 1] - cut[lz + 1][lx + 1];
    water = hm.water[lz + 1][lx + 1];
}

// Обход кольцами вокруг середины: сначала сама середина, потом края квадратов всё шире. Первое же
// подходящее место и берём - оно и будет ближайшим с точностью до шага.
template <class Fn>
bool ringScan(double cx, double cz, double step, int maxRing, Fn&& good, double& outX, double& outZ) {
    for (int ring = 0; ring <= maxRing; ++ring) {
        const int side = ring * 2;
        const int n = ring == 0 ? 1 : side * 4;
        for (int k = 0; k < n; ++k) {
            double dx = 0, dz = 0;
            if (ring != 0) {
                if (k < side) { dx = -ring + k; dz = -ring; }
                else if (k < side * 2) { dx = ring; dz = -ring + (k - side); }
                else if (k < side * 3) { dx = ring - (k - side * 2); dz = ring; }
                else { dx = -ring; dz = ring - (k - side * 3); }
            }
            const double x = cx + dx * step, z = cz + dz * step;
            if (good(x, z)) { outX = x; outZ = z; return true; }
        }
    }
    return false;
}

}  // namespace

Spawn findSpawn(const TerrainParams& p, double wantX, double wantZ, double maxBlocks) {
    const auto dryByRelief = [&](double x, double z) {
        const Relief r = reliefAt(p, int32_t(std::lround(x * VOXELS_PER_BLOCK)), int32_t(std::lround(z * VOXELS_PER_BLOCK)));
        return r.water == NO_WATER || r.water <= r.height;   // вода есть, только если она выше дна
    };
    double x = wantX, z = wantZ;
    bool moved = false;
    if (!dryByRelief(x, z)) {
        // Обходы от желаемой точки, от частого к редкому: сперва рядом мелким шагом (чтобы не
        // уехать за тридевять земель, когда берег в двух сотнях блоков), и только если вокруг
        // сплошная вода - шире и реже. Океаны здесь в десятки тысяч блоков, мелким шагом их не
        // перебрать, а редкий обход по ближним кольцам как раз и промахнулся бы мимо косы.
        struct Pass { double step, reach; };
        const Pass passes[3] = {{8.0, 512.0}, {64.0, 4096.0}, {768.0, maxBlocks}};
        bool found = false;
        for (const Pass& pass : passes) {
            if (pass.reach > maxBlocks) continue;
            double hx = x, hz = z;
            if (ringScan(wantX, wantZ, pass.step, int(pass.reach / pass.step), dryByRelief, hx, hz)) {
                x = hx;
                z = hz;
                found = true;
                break;
            }
        }
        if (!found) {   // суши нет совсем - ставим над водой в желаемой точке
            int32_t top = 0, water = 0;
            surfaceAt(p, int32_t(std::lround(wantX * VOXELS_PER_BLOCK)), int32_t(std::lround(wantZ * VOXELS_PER_BLOCK)), top, water);
            return {wantX, wantZ, double(std::max(top, water == NO_WATER ? top : water)) / VOXELS_PER_BLOCK, false, false};
        }
        moved = true;
    }
    // точный выбор столбца: рельеф воды не знает про срез земли у выхода шахты, а столбец знает
    const auto dryHonest = [&](double px, double pz) {
        int32_t top = 0, water = 0;
        surfaceAt(p, int32_t(std::lround(px * VOXELS_PER_BLOCK)), int32_t(std::lround(pz * VOXELS_PER_BLOCK)), top, water);
        return water == NO_WATER || water <= top;
    };
    double fx = x, fz = z;
    if (!dryHonest(fx, fz) && ringScan(x, z, 2.0, 16, dryHonest, fx, fz)) moved = true;
    int32_t top = 0, water = 0;
    surfaceAt(p, int32_t(std::lround(fx * VOXELS_PER_BLOCK)), int32_t(std::lround(fz * VOXELS_PER_BLOCK)), top, water);
    const bool dry = water == NO_WATER || water <= top;
    return {fx, fz, double(dry ? top : std::max(top, water)) / VOXELS_PER_BLOCK, moved, dry};
}

}  // namespace world
