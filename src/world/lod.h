#pragma once
// Дальние уровни рельефа (Р20): гладкие плитки по CDLOD вместо вокселей там, где воксель мельче точки экрана.
//
// Плитка уровня L - квадрат 32×32 шага по 2^L блоков (33×33 узла), выровнен по сетке своего уровня.
// Дальность уровня L - K·2^L блоков (K - «детальность»): чем дальше, тем реже узлы, примерно одинаково
// на экране. Высоты - из той же формулы, что воксели (Р2), без октав мельче шага (Р20).
//
// Морфинг: у узла хранится не только своя высота, но и та, что даст в этой точке уровень L+1. На краю
// дальности уровня нечётные узлы сползают к соседним чётным, высота - к высоте уровня L+1; полностью
// сползшая плитка совпадает с плиткой уровня L+1 точно - ни щелей, ни скачка при смене уровня.
#include <cstdint>
#include <vector>

#include "world/terrain.h"

namespace world {

constexpr int TILE = 32;              // шагов в плитке по стороне
constexpr int TILE_VERTS = TILE + 1;  // узлов по стороне
constexpr int TILE_WORDS = 5;         // 32-битных чисел на узел

struct TileKey {
    int32_t level, tx, tz;   // плитка уровня level с номером (tx, tz): начало - (tx, tz)·32·2^level блоков
    bool operator==(const TileKey&) const = default;
};
inline int64_t tileBlocks(int level) { return int64_t(TILE) << level; }
inline uint64_t tileHash(TileKey k) {
    return uint64_t(uint32_t(k.level)) << 58 ^ (uint64_t(uint32_t(k.tx)) & 0x1FFFFFFF) << 29 ^
           (uint64_t(uint32_t(k.tz)) & 0x1FFFFFFF);
}

// Узел - пять чисел:
//   0: высота дна (int16, воксели) | высота уровня L+1 в точке, куда узел сползает (int16) << 16
//   1: наклон по x | по z << 16 - int16, 1/256 вокселя на воксель (Р25: при 1/16 пологое дно шло полосами)
//   2: наклон уровня L+1 - так же
//   3: покров верх | под ним << 4 | верх уровня L+1 << 8 | под ним уровня L+1 << 12
//   4: поверхность воды (int16) | она же уровня L+1 (int16) << 16 - Р34, второй, прозрачный слой плитки. У сухого
//      узла рядом с водой - уровень соседней воды, но не выше дна без вокселя: треугольник берега - плоскость воды,
//      уходящая под землю, край воды - там, где земля её пересекает. У сухого узла без воды рядом - блок под землёй.
struct TileMesh {
    std::vector<uint32_t> data;   // TILE_VERTS² · TILE_WORDS, узел (i, j) - по номеру j·33 + i
    int32_t minH, maxH;           // воксели, по обоим уровням (дно и вода)
    bool water = false;           // есть ли вода хоть в одном узле (иначе слой воды не рисуется)
};
void buildTile(const TerrainParams& p, TileKey k, TileMesh& out);

struct LodSettings {
    int detail = 128;          // K: дальность уровня 0 в блоках; условие без щелей - K >= 65 (Р20)
    int drawDistance = 24576;  // блоков: гора в 15 км видна с равнины (решение пользователя)
    // зона вокселей (блоки): четверти плиток целиком внутри неё не рисуются, частично - дорезает шейдер
    int64_t holeMinX = 0, holeMinZ = 0, holeMaxX = 0, holeMaxZ = 0;
};
inline double lodRange(const LodSettings& s, int level) { return double(s.detail) * double(int64_t(1) << level); }
int lodTopLevel(const LodSettings& s);   // самый грубый уровень: его дальность покрывает всю видимость

// Что рисовать: плитка и какие её четверти (бит q: qx = q & 1, qz = q >> 1). Четверть грубой плитки
// рисуется, когда её место не заняли более мелкие плитки (классический выбор CDLOD).
struct TileDraw {
    TileKey key;
    uint8_t quadrants;
};
void selectTiles(const LodSettings& s, double camX, double camZ, std::vector<TileDraw>& out);

}  // namespace world
