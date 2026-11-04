#include "world/caves.h"

#include <algorithm>

#include "gen/hash.h"
#include "gen/noise.h"

namespace world {
namespace {

constexpr int STEP_LOG2 = 3;                 // узлы через 8 вокселей (полблока)
constexpr int STEP = 1 << STEP_LOG2;
// Узлов по стороне на кусок с каймой - ровно 7, как и сказано: начало куска с каймой всегда даёт
// остаток 7 по модулю 8, поэтому кайма задевает ещё один узел. Формула с +2 давала 6, и запись шла
// на узел дальше массива. Пока массив был низким, лишняя запись попадала в соседнюю строку того же
// массива и сходила с рук; стоило надстроить его вверх - она стала портить чужую память.
constexpr int MAX_NODES = (CHUNK + 2) / STEP + 3;
// По высоте узлов нужно больше: у куска под самой поверхностью кровля осыпается по пустоте,
// лежащей ниже него, и поле приходится знать на полосу опоры глубже (5 блоков = 80 вокселей).
constexpr int MAX_NODES_Y = (CHUNK + 2 + 10 * VOXELS_PER_BLOCK) / STEP + 5;

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

// Р79: на сколько шум входа не дотягивает до порога, а гашение уже ослаблено. Это ширина ската у
// края отверстия: в середине входа гашения нет, за этой полосой оно полное.
constexpr fx::q16 ENTRANCE_EDGE = fx::ONE / 4;

bool caveEntranceNear(const TerrainParams& p, int32_t cx, int32_t cz);

bool cavesHere(const TerrainParams& p, int32_t cx, int32_t cz) {
    for (int dz = -1; dz <= 1; ++dz)
        for (int dx = -1; dx <= 1; ++dx)
            if (caveEntranceNear(p, cx + dx, cz + dz)) return true;
    return false;
}

bool caveEntranceNear(const TerrainParams& p, int32_t cx, int32_t cz) {
    const CaveParams& c = p.caves;
    // Запас ТОТ ЖЕ, что у гашения хода у поверхности (ENTRANCE_EDGE). Пока здесь стояла одна
    // десятая, а гашение ослабевало уже с четверти, между ними лежала полоса: генератор землю у
    // входа срезал, а постройщик для соседнего столбца кусков пещеры не считал вовсе. По границе
    // таких столбцов земля оказывалась срезанной с одной стороны и целой с другой - отвесная
    // стенка, земляные столбы на её углах и снег, повисший над срезом.
    return gen::simplex2(cx * CHUNK - 1 + CHUNK / 2, cz * CHUNK - 1 + CHUNK / 2, c.entranceCellLog2, caveSeed(p.seed, ENTRANCE)) >=
           c.entranceFrom - ENTRANCE_EDGE;
}

// Р79: сколько опоры обязано быть над пустотой. Считается ОТ ПОДОШВЫ ЗЕМЛИ, а не от поверхности:
// на скалах и обрывах земли нет вовсе (покров сходит с крутизны), и там пещера имеет право
// начинаться почти сразу - это и есть вход в скале. А под лугом та же пещера обязана лежать ниже
// всей земли и ещё двух блоков породы, иначе своду не на чем держаться.
constexpr int32_t ROCK_ROOF = 2 * VOXELS_PER_BLOCK;
constexpr int32_t SOIL_DEPTH = 3 * VOXELS_PER_BLOCK + 1;   // покров и подпокровный слой
constexpr int32_t ROOF_BAND = SOIL_DEPTH + ROCK_ROOF;      // вся полоса опоры: пять блоков

bool caveAir(const TerrainParams& p, ChunkPos pos, const ColumnHeights& hm, uint64_t air[CHUNK + 2][CHUNK + 2],
             const uint8_t (*cut)[CHUNK + 2]) {
    const CaveParams& c = p.caves;
    const int32_t x0 = pos.x * CHUNK - 1, y0 = pos.y * CHUNK - 1, z0 = pos.z * CHUNK - 1;   // начало каймы

    // Быстрый отказ 1: кусок целиком в полосе опоры под поверхностью и над ним нет входа - вне входа
    // ходы туда не поднимаются вовсе, а залы лежат ещё глубже.
    int32_t maxDepth = INT32_MIN, minTop = INT32_MAX;
    for (int z = 0; z < CHUNK + 2; ++z)
        for (int x = 0; x < CHUNK + 2; ++x) {
            maxDepth = std::max(maxDepth, hm.h[z][x] - y0);
            minTop = std::min(minTop, hm.h[z][x]);
        }
    if (maxDepth < 0) return false;   // весь кусок над поверхностью
    const bool entranceArea =
        gen::simplex2(x0 + CHUNK / 2, z0 + CHUNK / 2, c.entranceCellLog2, caveSeed(p.seed, ENTRANCE)) >=
        c.entranceFrom - ENTRANCE_EDGE;
    if (maxDepth < ROOF_BAND && !entranceArea) return false;
    // Полоса опоры задевает этот кусок: значит в нём может осыпаться кровля, а сама пустота под ней
    // лежит ниже, в соседнем куске снизу - узлы приходится считать с запасом вниз.
    const bool bandHere = minTop - ROOF_BAND <= y0 + CHUNK + 1;
    const int32_t below = bandHere ? ROOF_BAND : 0;   // до низа полосы поиска: земля и два блока камня
    // и ВВЕРХ на толщу земли: поверхность столбца может лежать выше верха куска, а значение поля на
    // ней всё равно нужно - иначе обращение к узлам уйдёт за край массива и посчитается мусор
    const int32_t above = bandHere ? SOIL_DEPTH : 0;

    // узлы каналов
    const int32_t nx0 = x0 >> STEP_LOG2, ny0 = (y0 - below) >> STEP_LOG2, nz0 = z0 >> STEP_LOG2;
    const int nx = ((x0 + CHUNK + 1) >> STEP_LOG2) - nx0 + 2;
    const int ny = ((y0 + CHUNK + 1 + above) >> STEP_LOG2) - ny0 + 2;
    const int nz = ((z0 + CHUNK + 1) >> STEP_LOG2) - nz0 + 2;
    static thread_local Node nodes[MAX_NODES][MAX_NODES_Y][MAX_NODES];
    fx::q16 maxT = INT32_MIN, maxH = INT32_MIN;
    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                const Node n = caveNode(p, (nx0 + i) << STEP_LOG2, (ny0 + j) << STEP_LOG2, (nz0 + k) << STEP_LOG2);
                nodes[k][j][i] = n;
                maxT = std::max(maxT, n.tunnel);
                maxH = std::max(maxH, n.hall);
            }
    // Быстрый отказ 2: смешивание не выходит за пределы значений в узлах. Порог сдвинут на запас
    // воронки: при значении выше -SLOPE земля ещё осыпается, пусть пустоты и нет, и отказывать
    // тогда нельзя - иначе соседние куски разойдутся и над пустотой останется крышка.
    //
    // У куска, в котором лежит ВЕРХ столбца, порог много ниже: там считается срез земли, а он
    // тянется на четыре блока от кромки хода, и поле на таком удалении уже заметно ниже нуля. Пока
    // порог был общим, такой кусок отбрасывался, а нижний срез всё-таки считал: верхний рисовал
    // землю целиком, нижний срезал её под ним - пласт повисал, а под ним зияла дыра.
    const fx::q16 w2chunk = fx::mul(c.tunnelWidth, c.tunnelWidth);
    const fx::q16 rejectAt = bandHere ? 64 * w2chunk : 3 * w2chunk;
    if (maxT <= -rejectAt && maxH <= -rejectAt) return false;

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
            // покров хранится без каймы ([0..CHUNK-1]), а обход идёт с каймой - приводим индекс
            const int cvz = std::clamp(z - 1, 0, CHUNK - 1), cvx = std::clamp(x - 1, 0, CHUNK - 1);
            // толща земли этого столбца: ниже неё - порода
            const int32_t soil = hm.cover[cvz][cvx].soil;
            // До какой глубины ход гаснет у поверхности. Полоса ОДНА ДЛЯ ВСЕХ столбцов: своя толща
            // земли у столбца заметно рябит от соседа к соседу (её задаёт уклон местности), и если
            // подставить её сюда, рябь попадёт в само поле - а по нему считается срез. Тогда у
            // одного столбца срез нулевой, у соседнего полный, и по склону встают столбы в один
            // блок с нетронутым покровом наверху. Своя земля столбца только ограничивает срез.
            constexpr int32_t roofBand = ROOF_BAND;

            // смешивание поля по трём осям: x, затем z, затем y
            const auto triAt = [&](int32_t wy, fx::q16 ux, fx::q16 uz, auto get) {
                const int iy = (wy >> STEP_LOG2) - ny0;
                const fx::q16 fy = (wy & mask) << FR;
                const fx::q16 a = fx::lerp(get(nodes[iz][iy][ix]), get(nodes[iz][iy][ix + 1]), ux);
                const fx::q16 b = fx::lerp(get(nodes[iz + 1][iy][ix]), get(nodes[iz + 1][iy][ix + 1]), ux);
                const fx::q16 e = fx::lerp(get(nodes[iz][iy + 1][ix]), get(nodes[iz][iy + 1][ix + 1]), ux);
                const fx::q16 f = fx::lerp(get(nodes[iz + 1][iy + 1][ix]), get(nodes[iz + 1][iy + 1][ix + 1]), ux);
                return fx::lerp(fx::lerp(a, b, uz), fx::lerp(e, f, uz), fy);
            };
            const auto tri = [&](int32_t wy, auto get) { return triAt(wy, fx_, fz, get); };
            const fx::q16 w2 = fx::mul(c.tunnelWidth, c.tunnelWidth);   // толщина хода входит в поле квадратом
            // насколько поле должно не дотянуть до нуля, чтобы земля держалась вся: это и есть
            // ширина воронки - при трёх толщинах хода её край отходит примерно на две его ширины
            // Гашение вне входа обязано перекрывать ВСЮ ширину воронки (4 толщины хода), иначе в
            // полностью погашенном месте срез выходит не нулевым, а в три четверти блока - и там,
            // где постройщик кусков пещеры вообще не считает, соседи расходятся: дыры и столбы.
            const fx::q16 SLOPE = 4 * w2;
            // Р79: ГАШЕНИЕ У ПОВЕРХНОСТИ - ПЛАВНОЕ, а не «вход или не вход». Признак был двоичный: в
            // одном столбце ход не гасился вовсе и выходил наружу, а в соседнем гасился полностью, и
            // под поверхностью не оставалось пустоты ни на одном уровне. Край отверстия выходил
            // отвесной стеной в пять блоков. Теперь доля гашения идёт тем же шумом входа: в середине
            // входа - ноль, к его краю растёт до единицы, и кровля утончается постепенно, а край
            // отверстия получается скатом.
            const auto entranceDamp = [&]() -> fx::q16 {
                if (entrance < 0) {
                    fx::q16 v = fx::ONE;
                    // под водой и у берега входов нет: вода прозрачна (Р34, Р37), дыра в дне русла
                    // была бы видна насквозь
                    if (!hm.nearWater[z][x]) {
                        const fx::q16 e = gen::simplex2(wx, wz, c.entranceCellLog2, caveSeed(p.seed, ENTRANCE));
                        if (e >= c.entranceFrom) v = 0;
                        else if (e > c.entranceFrom - ENTRANCE_EDGE)
                            v = fx::q16(int64_t(c.entranceFrom - e) * fx::ONE / ENTRANCE_EDGE);
                    }
                    entrance = int(v);
                }
                return fx::q16(entrance);
            };

            // Поле хода с гашением у поверхности. Вне входа ход гаснет во всей полосе опоры: под
            // лугом пещера обязана лежать ниже земли и ещё двух блоков породы. У входа не гаснет -
            // там ход и должен выйти наружу.
            //
            // В полную силу гашение строгое: вычитаем заведомо больше, чем ход может дать
            // (наибольшее значение поля - это и есть толщина хода), поэтому вдали от входа в полосе
            // опоры пустоты не бывает совсем. Так куски у самой поверхности вне входов можно
            // отбрасывать целиком, не считая их.
            const auto field = [&](int32_t wy) {
                const int32_t d = top - wy;
                fx::q16 t = tri(wy, [](const Node& n) { return n.tunnel; });
                // В полную силу гашение обязано перекрывать и запас воронки (SLOPE), иначе вдали от
                // входа поле всё равно не дотягивало бы до нуля всего на две-три толщины хода, и
                // земля снималась бы частично по всей округе - целыми пластами.
                if (d < roofBand)
                    t -= fx::mul(SLOPE + fx::q16(int64_t(w2) * (roofBand - d + VOXELS_PER_BLOCK) / VOXELS_PER_BLOCK),
                                 entranceDamp());
                // зал считаем тем же полем, иначе земля над залом осталась бы висеть: пустота есть,
                // а величина, по которой решается осыпание, её не видит
                // Отсечка по наибольшему значению в узлах точна: смешивание за их пределы не
                // выходит, поэтому при maxH <= 0 зала в этом куске нет нигде. Срез земли считается
                // не здесь, а в soilCut, поэтому на согласие соседних кусков это не влияет.
                if (maxH > 0 && d >= c.hallDepth - 8 * VOXELS_PER_BLOCK) {
                    fx::q16 h = tri(wy, [](const Node& n) { return n.hall; });
                    // залы гаснут к верху на 8 блоках над hallDepth - потолок не плоский
                    if (d < c.hallDepth) h -= fx::q16(int64_t(fx::ONE * 3 / 10) * (c.hallDepth - d) / (8 * VOXELS_PER_BLOCK));
                    t = std::max(t, h);
                }
                return t;
            };

            // Пустота по самому полю: форму хода задаёт только шум, и ход идёт под тем углом, под
            // каким его провёл шум. Ничего вертикального здесь не прорезается.
            const auto natural = [&](int32_t wy) { return field(wy) > 0; };

            uint64_t bits = 0;
            for (int yi = 0; yi < CHUNK + 2; ++yi) {
                const int32_t wy = y0 + yi;
                if (wy > top) break;
                if (natural(wy)) bits |= uint64_t(1) << yi;
            }

            // срез земли у выхода шахты - тоже пустота (soilCut)
            if (cut && cut[z][x]) {
                const int32_t from = std::min(top, y0 + CHUNK + 1);
                for (int32_t wy = top - cut[z][x] + 1; wy <= from; ++wy) {
                    const int yi = int(wy - y0);
                    if (yi >= 0 && yi < CHUNK + 2) bits |= uint64_t(1) << yi;
                }
            }
            air[z][x] = bits;
            any |= bits != 0;
        }
    return any;
}

// ── Р79: срез земли у выхода шахты (см. caves.h) ───────────────────────────────────────────────
//
// Правило одно: НАД ПУСТОТОЙ ЛИБО КАМЕНЬ, ЛИБО НИЧЕГО. Нижняя грань хода, выходя к поверхности,
// срезает землю сама; над верхней грань земля без камня под собой держаться не может и осыпается,
// а по краю сходит на нет воронкой, повторяющей форму самой шахты.
//
// Задаёт всё это одна непрерывная величина: наибольшее значение поля хода в столбце от поверхности
// до двух блоков ниже подошвы земли. При нуле и выше земли нет вовсе, при недоборе в ширину воронки
// земля цела, между - соразмерно. Непрерывность тут - главное требование: любой порог, любое деление
// на скачущую величину, любая оглядка на соседа дают разрыв, а разрыв на земле выглядит отвесной
// стенкой, столбом в один блок или куском, висящим в воздухе.
void soilCut(const TerrainParams& p, int32_t chunkX, int32_t chunkZ, const ColumnHeights& hm,
             uint8_t cut[CHUNK + 2][CHUNK + 2]) {
    const CaveParams& c = p.caves;
    for (int z = 0; z < CHUNK + 2; ++z)
        for (int x = 0; x < CHUNK + 2; ++x) cut[z][x] = 0;

    const int32_t x0 = chunkX * CHUNK - 1, z0 = chunkZ * CHUNK - 1;
    // Доля гашения у поверхности - СВОЯ У КАЖДОГО СТОЛБЦА, и считается она здесь, до всего
    // остального. Проверять её в одной середине куска нельзя: сосед меряет в своей середине, и у
    // общего пограничного столбца ответы расходятся - один кусок землю срезает, другой нет, а на
    // стыке встаёт столб или зияет дыра. Заодно это и быстрый отказ: погашены все столбцы - считать
    // нечего.
    static thread_local fx::q16 dampCol[CHUNK + 2][CHUNK + 2];
    bool anyOpen = false;
    for (int z = 0; z < CHUNK + 2; ++z)
        for (int x = 0; x < CHUNK + 2; ++x) {
            fx::q16 v = fx::ONE;
            if (!hm.nearWater[z][x]) {
                const fx::q16 e = gen::simplex2(x0 + x, z0 + z, c.entranceCellLog2, caveSeed(p.seed, ENTRANCE));
                if (e >= c.entranceFrom) v = 0;
                else if (e > c.entranceFrom - ENTRANCE_EDGE)
                    v = fx::q16(int64_t(c.entranceFrom - e) * fx::ONE / ENTRANCE_EDGE);
            }
            dampCol[z][x] = v;
            anyOpen |= v < fx::ONE;
        }
    if (!anyOpen) return;
    // полоса, в которой вообще может что-то срезаться: от низа самой низкой земли до верха самой высокой
    const int32_t yLo = hm.minH - ROOF_BAND, yHi = hm.maxH;
    constexpr int MAX_Y = 160;   // узлов по высоте: 1280 вокселей = 80 блоков перепада на столбец кусков
    const int32_t nx0 = x0 >> STEP_LOG2, ny0 = yLo >> STEP_LOG2, nz0 = z0 >> STEP_LOG2;
    const int nx = ((x0 + CHUNK + 1) >> STEP_LOG2) - nx0 + 2;
    const int ny = (yHi >> STEP_LOG2) - ny0 + 2;
    const int nz = ((z0 + CHUNK + 1) >> STEP_LOG2) - nz0 + 2;
    if (ny > MAX_Y) return;   // непомерный перепад: воронку не рисуем, зато разнобоя не будет

    static thread_local Node nodes[MAX_NODES][MAX_Y][MAX_NODES];
    fx::q16 maxT = INT32_MIN, maxH = INT32_MIN;
    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                const Node n = caveNode(p, (nx0 + i) << STEP_LOG2, (ny0 + j) << STEP_LOG2, (nz0 + k) << STEP_LOG2);
                nodes[k][j][i] = n;
                maxT = std::max(maxT, n.tunnel);
                maxH = std::max(maxH, n.hall);
            }
    const fx::q16 w2 = fx::mul(c.tunnelWidth, c.tunnelWidth);
    // Ширина воронки в значениях поля. Восемь толщин пробовали - торец шахты от этого не смягчается
    // (там обрывается сам ход, и воронка идёт за ним), а покрова снимается в полтора раза больше.
    const fx::q16 wide = 4 * w2;
    const fx::q16 damp = wide + w2;     // гашение вне входа: заведомо перекрывает воронку
    // Отказа по maxT здесь быть НЕ МОЖЕТ: это наибольшее по узлам ЭТОГО куска, а сосед берёт свои
    // узлы и получает другое число. У общего пограничного столбца один сосчитал бы срез, а другой
    // отказался - и на стыке опять столб или дыра. Пусть считается, это недорого.
    (void)damp;

    constexpr int32_t mask = STEP - 1;
    constexpr int FR = fx::SHIFT - STEP_LOG2;
    for (int z = 0; z < CHUNK + 2; ++z)
        for (int x = 0; x < CHUNK + 2; ++x) {
            const int32_t wx = x0 + x, wz = z0 + z, top = hm.h[z][x];
            const int32_t soil = hm.soilThick[z][x];   // с каймой: одно число у обоих соседей
            if (soil <= 0) continue;
            const int ix = (wx >> STEP_LOG2) - nx0, iz = (wz >> STEP_LOG2) - nz0;
            const fx::q16 fx_ = (wx & mask) << FR, fz = (wz & mask) << FR;
            const fx::q16 dampK = dampCol[z][x];
            if (dampK >= fx::ONE) continue;   // вне входа срез строго нулевой

            const auto field = [&](int32_t wy) {
                const int iy = (wy >> STEP_LOG2) - ny0;
                const fx::q16 fy = (wy & mask) << FR;
                const auto get = [](const Node& n) { return n.tunnel; };
                const fx::q16 a = fx::lerp(get(nodes[iz][iy][ix]), get(nodes[iz][iy][ix + 1]), fx_);
                const fx::q16 b = fx::lerp(get(nodes[iz + 1][iy][ix]), get(nodes[iz + 1][iy][ix + 1]), fx_);
                const fx::q16 e = fx::lerp(get(nodes[iz][iy + 1][ix]), get(nodes[iz][iy + 1][ix + 1]), fx_);
                const fx::q16 f = fx::lerp(get(nodes[iz + 1][iy + 1][ix]), get(nodes[iz + 1][iy + 1][ix + 1]), fx_);
                fx::q16 t = fx::lerp(fx::lerp(a, b, fz), fx::lerp(e, f, fz), fy);
                const int32_t d = top - wy;
                if (d < ROOF_BAND)
                    t -= fx::mul(damp + fx::q16(int64_t(w2) * (ROOF_BAND - d) / VOXELS_PER_BLOCK), dampK);
                // зал - такая же пустота: без него земля над залом осталась бы висеть крышкой
                if (maxH > 0 && d >= c.hallDepth - 8 * VOXELS_PER_BLOCK) {
                    const auto geth = [](const Node& n) { return n.hall; };
                    const fx::q16 ha = fx::lerp(geth(nodes[iz][iy][ix]), geth(nodes[iz][iy][ix + 1]), fx_);
                    const fx::q16 hb = fx::lerp(geth(nodes[iz + 1][iy][ix]), geth(nodes[iz + 1][iy][ix + 1]), fx_);
                    const fx::q16 he = fx::lerp(geth(nodes[iz][iy + 1][ix]), geth(nodes[iz][iy + 1][ix + 1]), fx_);
                    const fx::q16 hf = fx::lerp(geth(nodes[iz + 1][iy + 1][ix]), geth(nodes[iz + 1][iy + 1][ix + 1]), fx_);
                    fx::q16 h = fx::lerp(fx::lerp(ha, hb, fz), fx::lerp(he, hf, fz), fy);
                    if (d < c.hallDepth) h -= fx::q16(int64_t(fx::ONE * 3 / 10) * (c.hallDepth - d) / (8 * VOXELS_PER_BLOCK));
                    t = std::max(t, h);
                }
                return t;
            };

            // вдоль высоты поле кусочно-линейно между узлами, поэтому наибольшее достигается только
            // в узле или на конце полосы - выборка точная
            const int32_t lo = std::max(top - SOIL_DEPTH - ROCK_ROOF, yLo);
            fx::q16 tb = std::max(field(top), field(lo));
            for (int32_t wy = (top & ~int32_t(STEP - 1)); wy > lo; wy -= STEP) tb = std::max(tb, field(wy));

            int32_t v = tb >= 0 ? soil
                                : std::min<int32_t>(soil, int32_t(std::max<int64_t>(
                                      0, int64_t(SOIL_DEPTH) * (int64_t(wide) + tb) / wide)));
            // стык с целой землёй - касательный: последний блок сводится к нулю по квадрату
            if (v > 0 && v < VOXELS_PER_BLOCK) v = v * v / VOXELS_PER_BLOCK;
            cut[z][x] = uint8_t(v);
        }
}

}  // namespace world
