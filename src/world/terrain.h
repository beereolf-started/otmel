#pragma once
// Рельеф по карте высот (Р17) и биомы (Р19): форма рельефа и климат - раздельно.
// Форма: три вида высоты (равнины - Перлин, холмы - симплекс, горы - хребты), смешанные непрерывными весами
// по медленному шуму «рельефности» - без ступеньки на границе по построению.
// Климат (по образцу TFC): температура - пояса по z и падение с высотой, осадки - пояса по x; он задаёт покров.
#include <cstdint>
#include <vector>

#include "gen/fixed.h"
#include "world/chunk.h"

namespace world {

constexpr fx::q16 DEG = fx::ONE;   // градус Цельсия и миллиметр осадков - в Q16

// Пещеры (Р26), см. world/caves.h. Глубины и высоты - в вокселях.
struct CaveParams {
    int tunnelCellLog2 = 10;                          // тоннели: шум с ячейкой 64 блока - ходы радиусом ~2 блока
    fx::q16 tunnelWidth = fx::ONE * 9 / 100;          // толщина: |a|, |b| < 0.09
    int32_t tunnelCover = 4 * VOXELS_PER_BLOCK;       // ближе к поверхности - только у входов
    int entranceCellLog2 = 12;                        // входы: 2D-шум с ячейкой 256 блоков
    fx::q16 entranceFrom = fx::ONE * 55 / 100;      // входы открыты на ~7% площади
    int hallCellLog2 = 11;                            // залы: ячейка 128 блоков
    fx::q16 hallFrom = fx::ONE * 42 / 100;
    int32_t hallDepth = 24 * VOXELS_PER_BLOCK;        // залы - не ближе 24 блоков к поверхности
    int giantCellLog2 = 13;                           // ярус гигантских полостей: ячейка 512 блоков
    fx::q16 giantFrom = fx::ONE * 30 / 100;
    int32_t giantTop = -150 * VOXELS_PER_BLOCK, giantFull = -200 * VOXELS_PER_BLOCK;
};

struct TerrainParams {
    uint32_t seed = 1337;
    // равнины (Р15, Р17) - прежняя формула без изменений
    int32_t baseHeight = 32 * VOXELS_PER_BLOCK;   // средняя высота: 32 блока, в вокселях
    int32_t amplitude = 20 * VOXELS_PER_BLOCK;    // размах: ±20 блоков
    int firstCellLog2 = 11;                        // первая октава - 2048 вокселей = 128 блоков
    int octaves = 6;                               // мельчайшая - 64 вокселя = 4 блока
    // холмы - симплекс: 44 ± 36 блоков, крупнейшая волна 256 блоков, мельчайшая 8
    int32_t hillsBase = 44 * VOXELS_PER_BLOCK, hillsAmp = 36 * VOXELS_PER_BLOCK;
    int hillsCellLog2 = 12, hillsOctaves = 6;
    // горы - хребты: от 34 блоков (дно долин) до 234 (гребни), крупнейший хребет 512 блоков, мельчайший 8
    int32_t mountBase = 34 * VOXELS_PER_BLOCK, mountAmp = 200 * VOXELS_PER_BLOCK;
    int mountCellLog2 = 13, mountOctaves = 7;
    // рельефность - маска на шуме значений: ячейка 1024 блока, 3 октавы; пороги - по долям площади (Р19)
    int reliefCellLog2 = 14, reliefOctaves = 3;
    fx::q16 hillsFrom = fx::ONE * 18 / 100, hillsTo = fx::ONE * 34 / 100; // равнины -> холмы (Р24: равнины - большие)
    fx::q16 mountFrom = fx::ONE * 2 / 10, mountTo = fx::ONE * 4 / 10;   // холмы -> горы (вид 0)
    // Вид гор (Р23): 0 - пятна по рельефности, острые хребты; 1 - цепи вдоль поясов, хребты прежние;
    // 2 - цепи + гладкий массив поперёк цепи + пологие гребни + «колено» по высоте
    int mountainStyle = 2;
    // пояс цепи: b = 1 - |шум| (ячейка 4096 блоков, 2 октавы), 1 - ось цепи; горы - где b выше chainFrom
    int beltCellLog2 = 17, beltOctaves = 2;
    fx::q16 chainFrom = fx::ONE * 62 / 100, chainTo = fx::ONE * 80 / 100;
    fx::q16 footWidth = fx::ONE * 15 / 100;                             // предгорья-холмы по обе стороны цепи
    // присутствие цепей (Р24): свой медленный шум, ячейка 8192 блоков - переходы в километры (конец цепи оседает,
    // а не обрывается), и большие области вовсе без гор
    int chainCellLog2 = 17, chainOctaves = 2;
    fx::q16 gateFrom = fx::ONE * 15 / 100, gateTo = fx::ONE * 45 / 100;
    fx::q16 mountGain = fx::ONE * 42 / 100;                             // размах следующей октавы гребней
    // масштаб - монументальный (решение пользователя: горы в 500 блоков, высокогорье - до километра);
    // нёрфится острота, а не рост: гребни без квадрата и «колено» над peakKnee
    int32_t massifAmp = 220 * VOXELS_PER_BLOCK, ridgeAmp = 260 * VOXELS_PER_BLOCK;
    int32_t peakKnee = 420 * VOXELS_PER_BLOCK;                          // выше - каждый блок даёт половину
    // высокогорье: редкие области (~10 км), где цепи шире и вдвое выше - до ~950 блоков (предел мира Р10 - 1000)
    int upliftCellLog2 = 18, upliftOctaves = 2;
    fx::q16 upliftFrom = fx::ONE * 30 / 100, upliftTo = fx::ONE * 55 / 100;
    fx::q16 upliftWiden = fx::ONE * 17 / 100;                           // на столько ниже порог пояса - цепь шире
    // моря и плато (Р25). Материковость - медленный шум: ниже oceanFrom - океан, выше oceanTo - суша, между -
    // берег (тем же смешением, без ступеньки). Суша - на landRise выше моря. Р36: ячейка 2^22 (~262 км), 5 октав -
    // материки и океаны в сотни километров, берег изрезан до 16 км (заливы, острова у берега); было 16 км, 3
    // октавы - весь мир архипелагом, реки не длиннее нескольких километров
    int continentCellLog2 = 22, continentOctaves = 5;
    // континенты - в океане, а не океаны в континентах: суши ~30% (как на Земле); порог - по распределению
    // материковости (квантиль 70% - 0.19), берег - от квантиля 65% до 75%
    fx::q16 oceanFrom = fx::ONE * 14 / 100, oceanTo = fx::ONE * 24 / 100;
    int32_t landRise = 16 * VOXELS_PER_BLOCK;          // равнины суши - в среднем на 16 блоков выше моря
    int32_t oceanDepth = 40 * VOXELS_PER_BLOCK;        // дно океана - на 40 блоков ниже моря (± 6)
    // плато: медленный шум (ячейка 4096 блоков) поднимает ВЕСЬ рельеф суши на 0..plateauRise - широкие
    // спокойные равнины на разных высотах, переходы между ними - километры
    int plateauCellLog2 = 16, plateauOctaves = 2;
    fx::q16 plateauFrom = -fx::ONE * 15 / 100, plateauTo = fx::ONE * 45 / 100;
    int32_t plateauRise = 90 * VOXELS_PER_BLOCK;
    // климат
    int32_t climateBand = 20000 * VOXELS_PER_BLOCK;   // от самого холодного пояса до самого жаркого (как в TFC)
    fx::q16 tempMid = 8 * DEG, tempSwing = 22 * DEG;  // на уровне моря: от -14 до +30 °C по поясам
    fx::q16 tempNoise = 5 * DEG;                       // местные отклонения ±5 °C
    fx::q16 rainMid = 250 * DEG, rainSwing = 250 * DEG, rainNoise = 75 * DEG;   // осадки 0..500 мм в год
    int climateCellLog2 = 15, climateOctaves = 2;      // шум климата: ячейка 2048 блоков
    int32_t seaLevel = 32 * VOXELS_PER_BLOCK;          // уровень моря (Р25); от него же - падение температуры с высотой
    fx::q16 lapsePerVoxel = 184;                       // 0.045 °C на блок: снег в умеренном поясе - с ~220 блоков
    fx::q16 snowBelow = -2 * DEG;                      // снег - где средняя температура ниже
    fx::q16 desertRain = 100 * DEG, desertTemp = 15 * DEG;   // пустыня: сухо и тепло
    CaveParams caves;
};

// Медленные поля (Р22): рельефность и шумы климата меняются на сотнях блоков - их считаем в узлах глобальной
// сетки через 2^SLOW_STEP_LOG2 вокселей (4 блока) и смешиваем между узлами линейно. Узлы глобальные: значение
// в точке не зависит от того, кто его спросил (кусок, плитка, карта) - шва нет.
constexpr int SLOW_STEP_LOG2 = 6;
struct SlowFields {
    fx::q16 relief, temp, rain;   // рельефность; шумы температуры и осадков (без поясов - те точные)
    fx::q16 belt;                 // пояс горной цепи (Р23): 1 на оси цепи, к краям падает
    fx::q16 uplift;               // поднятие (Р23): шум высокогорья
    fx::q16 chain;                // присутствие цепей (Р24)
    fx::q16 continent, plateau;   // материковость и плато (Р25)
};
SlowFields slowAt(const TerrainParams& p, int32_t x, int32_t z, int minCellLog2 = 0);
// Для плотной выборки: узлы прямоугольника [x0..x1]×[z0..z1] (воксели, включительно) - по одному разу
struct SlowGrid {
    int32_t nx0 = 0, nz0 = 0, w = 0, h = 0;
    std::vector<SlowFields> nodes;
    void build(const TerrainParams& p, int32_t x0, int32_t z0, int32_t x1, int32_t z1, int minCellLog2 = 0);
    SlowFields at(int32_t x, int32_t z) const;   // то же, что slowAt, до бита
};

// Форма рельефа в столбце: высота поверхности (верхний твёрдый воксель) и веса трёх видов, сумма весов = 1
struct Relief {
    int32_t height;
    fx::q16 wPlains, wHills, wMount;   // веса видов рельефа СУШИ (их сумма = 1)
    fx::q16 land;                      // доля суши: 0 - океан, 1 - суша, между - берег (Р25)
    int32_t water = INT32_MIN;         // Р28: верхний воксель воды (море, река, озеро) или NO_WATER
    bool shore = false;                // Р29: у воды - берег или дно русла (песок)
};
struct RiverSet;
// minCellLog2 - для дальних уровней (Р20): октавы мельче 2^minCellLog2 вокселей не считаются.
// Итоговый рельеф: исходный + море + реки и озёра (Р29). rivers - звенья рек, собранные на кусок или плитку
// (world/rivers.h); без них (первый вариант) - собираются на одну точку (медленнее, для карт и проверок).
Relief reliefAt(const TerrainParams& p, int32_t x, int32_t z, int minCellLog2 = 0);
Relief reliefAt(const TerrainParams& p, int32_t x, int32_t z, const SlowFields& slow, int minCellLog2,
                const RiverSet* rivers);
// Исходный рельеф - без воды и рек (по нему же реки и текут)
Relief baseReliefAt(const TerrainParams& p, int32_t x, int32_t z, const SlowFields& slow, int minCellLog2 = 0);

// Климат столбца на уровне моря: средняя годовая температура (°C) и осадки (мм в год), оба в Q16
struct Climate {
    fx::q16 temp, rain;
};
Climate climateAt(const TerrainParams& p, int32_t x, int32_t z);
Climate climateAt(const TerrainParams& p, int32_t x, int32_t z, const SlowFields& slow);
fx::q16 temperatureAt(const TerrainParams& p, const Climate& c, int32_t height);   // с поправкой на высоту

// Покров: верхний воксель и слой под ним (3 блока)
struct Cover {
    Material top, under;
};
Cover coverAt(const TerrainParams& p, const Climate& c, int32_t height);

// Материал нетронутого рельефа - формулой от высоты поверхности и покрова столбца (Р2, Р22): выше поверхности
// воздух, в верхнем вокселе покров, под ним 3 блока подпокровного слоя, ниже камень. Его же пишет fillChunk;
// сетке нетронутого куска массив вокселей не нужен вовсе - материал нужен только у видимых граней.
// Р28: выше дна до уровня воды столбца (water - верхний воксель воды, NO_WATER - сухо) - вода.
constexpr int32_t NO_WATER = INT32_MIN;
inline Material columnMaterial(int32_t top, Cover cv, int32_t wy, int32_t water = NO_WATER) {
    if (wy > top) return wy <= water ? WATER : AIR;
    if (wy == top) return cv.top;
    if (wy >= top - 3 * VOXELS_PER_BLOCK) return cv.under;
    return STONE;
}

// Высоты столбцов куска С КАЙМОЙ в один воксель: [-1..32] по x и z, 34×34. Кайма нужна сетке: грань на краю
// куска видна, только если сосед за краем - воздух, а соседа считаем той же формулой, не трогая чужой кусок.
struct ColumnHeights {
    int32_t h[CHUNK + 2][CHUNK + 2];   // [z+1][x+1], высота дна (верхний воксель грунта), воксели
    int32_t water[CHUNK + 2][CHUNK + 2];   // Р28: верхний воксель воды столбца или NO_WATER
    Cover cover[CHUNK][CHUNK];         // [z][x], только без каймы: покров нужен лишь своим вокселям
    int32_t minH, maxH;                // Р37: нижняя - по дну (вода прозрачна - дно под ней видно), верхняя - по
                                       // поверхности max(дно, вода)
    bool nearWater[CHUNK + 2][CHUNK + 2];   // Р37: вода над дном или берег - здесь нет входов в пещеры
};

// Уровень воды столбца (Р28) - поле, как высота рельефа: вода не одна на мир, а там, где её даёт биом.
// Море - в океанических областях (доля суши < 1); реки и озёра - Р29; низина ниже уровня моря на материке - сухая.
// Верхний воксель воды или NO_WATER; вода есть, только если он выше дна.
int32_t waterLevelAt(const TerrainParams& p, const Relief& r);

void columnHeights(const TerrainParams& p, int32_t chunkX, int32_t chunkZ, ColumnHeights& out);

// Заполнить кусок по высотам: камень, 3 блока подпокровного слоя, сверху воксель покрова
void fillChunk(const ColumnHeights& heights, Chunk& c);

// Контрольная сумма вокселей куска - для проверки воспроизводимости (Р2)
uint64_t checksum(const Chunk& c);

}  // namespace world
