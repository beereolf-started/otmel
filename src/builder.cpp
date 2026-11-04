#include "builder.h"

#include <algorithm>
#include <chrono>

#include "world/caves.h"
#include "world/ores.h"
#include "world/mesher.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

namespace {
using clk = std::chrono::steady_clock;

double msSince(clk::time_point t) { return std::chrono::duration<double, std::milli>(clk::now() - t).count(); }

// Р88: КАМУШЕК ВКРАПЛЕНИЯ. Не кубик: форма такая же неправильная, как у кроны дерева, но
// вытянутая вдоль складки - полуось a идёт по направлению dir, b поперёк, c по вертикали и самая
// малая. Края рваные: порог сравнения качается своим хешем на каждый мелкий воксель.
//
// Грани ставятся только там, где соседний мелкий воксель пуст, - внутренних граней у камушка нет.
void addSpeck(std::vector<uint32_t>& out, int lx, int ly, int lz, const world::OreSpeck& sp, int ore, int& quads) {
    const float ang = float(sp.dir) * 0.39269908f;   // 2π / 16
    const float ca = std::cos(ang), sa = std::sin(ang);
    const float ia = 1.0f / float(sp.a), ib = 1.0f / float(sp.b), ic = 1.0f / float(sp.c);
    const auto h1 = [&](int dx, int dy, int dz) {
        uint32_t h = uint32_t(sp.x + dx) * 0x8DA6B343u ^ uint32_t(sp.y + dy) * 0xD8163841u ^
                     uint32_t(sp.z + dz) * 0xCB1AB31Fu ^ sp.roll;
        h ^= h >> 15; h *= 0x2C1B3C6Du; h ^= h >> 12; h *= 0x297A2D39u; h ^= h >> 15;
        return h;
    };
    const auto inside = [&](int dx, int dy, int dz) {
        if (!sp.a) return dx == 0 && dy == 0 && dz == 0;   // крупинка - один мелкий воксель
        const float u = float(dx) * ca + float(dz) * sa, v = -float(dx) * sa + float(dz) * ca;
        const float r = u * ia * u * ia + v * ib * v * ib + float(dy) * ic * float(dy) * ic;
        const float jit = float(h1(dx, dy, dz) & 1023u) / 1023.0f;
        return r <= 1.0f + 0.6f * (jit - 0.5f);
    };
    const int ra = sp.a ? int(sp.a) + 1 : 0, rc = sp.c ? int(sp.c) + 1 : 0;
    static const int AX[3][3] = {{0, 1, 2}, {1, 2, 0}, {2, 0, 1}};
    for (int dz = -ra; dz <= ra; ++dz)
        for (int dy = -rc; dy <= rc; ++dy)
            for (int dx = -ra; dx <= ra; ++dx) {
                if (!inside(dx, dy, dz)) continue;
                // оттенок палитры: основной чаще всего, блестящий - редкость, он и оживляет камушек
                const uint32_t t = h1(dx, dy, dz) >> 11 & 7u;
                const int shade = t < 4u ? 1 : (t < 7u ? 0 : 2);
                const uint32_t m = uint32_t(world::ORE_FIRST + ore * world::ORE_SHADES + shade);
                const int lo[3] = {lx + dx, ly + dy, lz + dz};
                const int nb[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
                for (int ai = 0; ai < 3; ++ai)
                    for (int sign = 0; sign < 2; ++sign) {
                        const int f = ai * 2 + sign;
                        if (inside(dx + nb[f][0], dy + nb[f][1], dz + nb[f][2])) continue;   // грань внутри тельца
                        const int d = AX[ai][0], u = AX[ai][1], v = AX[ai][2];
                        int p[4][3];
                        for (int k = 0; k < 4; ++k) {
                            p[k][d] = lo[d] + (sign == 0 ? 1 : 0);
                            p[k][u] = lo[u] + ((k == 1 || k == 2) ? 1 : 0);
                            p[k][v] = lo[v] + (k >= 2 ? 1 : 0);
                        }
                        const int o0[4] = {0, 1, 2, 3}, o1[4] = {0, 3, 2, 1};
                        const int* o = sign == 0 ? o0 : o1;
                        for (int k = 0; k < 4; ++k)
                            out.push_back(uint32_t(p[o[k]][0]) | uint32_t(p[o[k]][1]) << 6 |
                                          uint32_t(p[o[k]][2]) << 12 | uint32_t(f) << 18 | m << 21);
                        ++quads;
                    }
            }
}

}  // namespace

Builder::Builder(const world::TerrainParams& tp) : tp_(tp) {
    const unsigned n = std::max(1u, std::thread::hardware_concurrency() - 1);   // главный поток рисует
    for (unsigned i = 0; i < n; ++i) threads_.emplace_back(&Builder::worker, this);
}

Builder::~Builder() {
    {
        std::lock_guard lk(mu_);
        stop_ = true;
    }
    cv_.notify_all();
    for (auto& t : threads_) t.join();
}

void Builder::setColumnJobs(std::vector<ColumnJob> jobs) {
    {
        std::lock_guard lk(mu_);
        jobs.erase(std::remove_if(jobs.begin(), jobs.end(),
                                  [&](const ColumnJob& j) { return buildingColumns_.count(colKey(j.cx, j.cz)) > 0; }),
                   jobs.end());
        columns_ = std::move(jobs);
    }
    cv_.notify_all();
}

void Builder::requestTiles(const std::vector<world::TileKey>& keys) {
    bool added = false;
    {
        std::lock_guard lk(mu_);
        for (const world::TileKey& k : keys)
            if (wantedTiles_.insert(world::tileHash(k)).second) { tiles_.push_back(k); added = true; }
    }
    if (added) cv_.notify_all();
}

bool Builder::popColumn(ColumnResult& out) {
    std::lock_guard lk(mu_);
    if (doneColumns_.empty()) return false;
    out = std::move(doneColumns_.front());
    doneColumns_.pop_front();
    return true;
}

bool Builder::popTile(TileResult& out) {
    std::lock_guard lk(mu_);
    if (doneTiles_.empty()) return false;
    out = std::move(doneTiles_.front());
    doneTiles_.pop_front();
    wantedTiles_.erase(world::tileHash(out.key));
    return true;
}

void Builder::worker() {
    // Строители - ниже обычного приоритета: на 4 ядрах (8 потоков) семь строителей на равных отнимали время
    // у потока кадра - рывки до десятков мс в любой его части. Теперь кадр всегда первый, стройка - остаток.
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#endif
    world::ColumnHeights hm;
    std::vector<uint32_t> verts;
    static thread_local uint64_t air[world::CHUNK + 2][world::CHUNK + 2];
    // на сколько срезана земля в каждом столбце: по этому числу покров ложится на новый верх
    static thread_local uint8_t cut[world::CHUNK + 2][world::CHUNK + 2];
    for (;;) {
        bool isTile = false;
        world::TileKey tile{};
        ColumnJob col{};
        {
            std::unique_lock lk(mu_);
            cv_.wait(lk, [&] { return stop_ || !tiles_.empty() || !columns_.empty(); });
            if (stop_) return;
            if (!tiles_.empty()) {
                // плитки - первыми, из них - самая грубая (её четверть закрывает место, пока нет мелких)
                auto it = std::max_element(tiles_.begin(), tiles_.end(),
                                           [](const world::TileKey& a, const world::TileKey& b) { return a.level < b.level; });
                tile = *it;
                *it = tiles_.back();
                tiles_.pop_back();
                isTile = true;
            } else {
                col = columns_.back();
                columns_.pop_back();
                buildingColumns_.insert(colKey(col.cx, col.cz));
            }
        }
        const auto t0 = clk::now();
        if (isTile) {
            TileResult r{tile, {}, 0};
            world::buildTile(tp_, tile, r.mesh);
            r.ms = msSince(t0);
            std::lock_guard lk(mu_);
            doneTiles_.push_back(std::move(r));
            continue;
        }
        ColumnResult r{col.cx, col.cz, col.cyMin, col.caps, {}, 0, 0};
        world::columnHeights(tp_, col.cx, col.cz, hm);
        // Срез земли у выхода шахты - ДО выбора кусков: он опускает поверхность до трёх блоков, и
        // нижний строящийся кусок надо выбирать уже по сниженной высоте. Иначе срез уводит
        // поверхность в кусок, который не строится вовсе, и под срезанной землёй остаётся
        // непостроенная пустота - обрубок земли, сквозь который видно небо.
        //
        // Считается для ВСЕХ столбцов кусков, а не только там, где рядом вход: иначе сосед без входа
        // среза не знает, считает пограничный столбец полной высоты и не рисует его боковую грань.
        // Где гашение полное во всех столбцах, расчёт выходит сразу и ничего не стоит.
        // Р83: снимок правок берётся ОДИН на столбец: во время постройки он меняться не должен,
        // иначе маска и материалы разойдутся между соседними кусками одного столбца.
        const std::shared_ptr<const world::EditMap> ed = edits_ ? edits_->snapshot() : nullptr;
        const world::EditMap* edits = ed && !ed->empty() ? ed.get() : nullptr;
        r.editStamp = ed ? ed->size() : 0;
        world::soilCut(tp_, col.cx, col.cz, hm, cut);
        int32_t maxCut = 0;
        for (int z = 0; z < world::CHUNK + 2; ++z)
            for (int x = 0; x < world::CHUNK + 2; ++x) maxCut = std::max<int32_t>(maxCut, cut[z][x]);
        // по высоте - куски, через которые проходит поверхность (с каймой), Р2; рядом с камерой - и ниже, до cyMin:
        // там строятся только куски с пещерами (сплошной камень граней не даёт), Р26
        int cyLo = (hm.minH - maxCut - 1) >> world::CHUNK_LOG2, cyHi = hm.maxH >> world::CHUNK_LOG2;
        // Р83: СЛОМАННЫЙ БЛОК МОЖЕТ ЛЕЖАТЬ НИЖЕ ОБЫЧНОГО ДНА СТОЛБЦА. Копая вниз, игрок быстро
        // выходит за куски, которые строятся по рельефу, - и дальше грани переставали открываться:
        // куска, в котором появилась пустота, просто не существовало. Поэтому ярус кусков
        // раздвигается до самой глубокой правки столбца (с каймой: правка у границы открывает
        // грань соседу).
        int editLo = INT32_MAX, editHi = INT32_MIN;
        if (edits) {
            const int32_t bx0 = col.cx * world::CHUNK - 1, bx1 = col.cx * world::CHUNK + world::CHUNK;
            const int32_t bz0 = col.cz * world::CHUNK - 1, bz1 = col.cz * world::CHUNK + world::CHUNK;
            for (const auto& [k, e] : *edits) {
                if (e.x < bx0 || e.x > bx1 || e.z < bz0 || e.z > bz1) continue;
                const int cy = e.y >> world::CHUNK_LOG2;
                editLo = std::min(editLo, cy);
                editHi = std::max(editHi, cy);
            }
        }
        if (editLo != INT32_MAX) { cyLo = std::min(cyLo, editLo); cyHi = std::max(cyHi, editHi); }
        constexpr int32_t WORLD_BOTTOM = -500 * world::VOXELS_PER_BLOCK / world::CHUNK;   // Р10
        // Р39: LOD пещер. Вне ближнего яруса - пещеры с входами до SHALLOW вглубь, ниже - закрыты дном (вход виден
        // провалом с дном, а не дырой в пустоту). Пол столбца - гладко по рельефу (без рек, крупными формами)
        // минус SHALLOW; пещера обрывается не ниже наибольшего пола из столбца и 8 соседей: каждый сосед построен
        // вниз до своего пола, значит до этого уровня - и стенки пещеры у границы столбцов есть с обеих сторон.
        constexpr int32_t SHALLOW = 24 * world::VOXELS_PER_BLOCK;
        int32_t caveFloor = INT32_MIN;
        int cyShallow = cyLo;
        // закрытые тоннели без входа сверху не видны - вне ближнего яруса пещеры только у столбцов рядом со входом
        // (вход в столбце или у соседа); со стороны соседа без пещер - заглушка: у границы пещера закрыта своей
        // стенкой (у соседа там сплошной камень без граней - иначе пустота видна насквозь)
        const auto hasCaves = [&](int32_t cx, int32_t cz) { return world::cavesHere(tp_, cx, cz); };
        bool shallowCaves = true;
        uint8_t shallowCaps = 0;
        if (col.cyMin == NO_DEPTH) {
            shallowCaves = hasCaves(col.cx, col.cz);
            if (shallowCaves)
                shallowCaps = uint8_t((hasCaves(col.cx - 1, col.cz) ? 0 : CAP_NX) | (hasCaves(col.cx + 1, col.cz) ? 0 : CAP_PX) |
                                      (hasCaves(col.cx, col.cz - 1) ? 0 : CAP_NZ) | (hasCaves(col.cx, col.cz + 1) ? 0 : CAP_PZ));
        }
        if (col.cyMin == NO_DEPTH && shallowCaves) {
            const auto floorOf = [&](int32_t cx, int32_t cz) {
                const int32_t x = cx * world::CHUNK + world::CHUNK / 2, z = cz * world::CHUNK + world::CHUNK / 2;
                return world::baseReliefAt(tp_, x, z, world::slowAt(tp_, x, z, 7), 7).height - SHALLOW;
            };
            const int32_t own = floorOf(col.cx, col.cz);
            caveFloor = own;
            for (int dz = -1; dz <= 1; ++dz)
                for (int dx = -1; dx <= 1; ++dx)
                    if ((dx || dz) && hasCaves(col.cx + dx, col.cz + dz)) caveFloor = std::max(caveFloor, floorOf(col.cx + dx, col.cz + dz));
            cyShallow = std::min(cyLo, own >> world::CHUNK_LOG2);
        }
        // Р79: срез земли у выхода шахты - ОДИН НА ВЕСЬ СТОЛБЕЦ КУСКОВ, до постройки. Пока он
        // считался внутри куска, два куска одного столбца могли решить по-разному, и на стыке
        // вставала стенка или повисал кусок земли.
        const int cyFrom = col.cyMin == NO_DEPTH ? cyShallow : std::max(std::min(cyLo, col.cyMin), WORLD_BOTTOM);
        // Р88: вкрапления руды собираются по кускам (здесь же известна пустота пещер) и
        // выкладываются мелкими сетками уже после цикла.
        static thread_local std::vector<world::OreSpeck> specks;
        static thread_local std::unordered_map<uint64_t, std::vector<std::pair<world::OreSpeck, uint8_t>>> cells;
        cells.clear();
        const auto fd = [](int32_t a, int32_t b) { return a >= 0 ? a / b : -((-a + b - 1) / b); };
        for (int cy = cyFrom; cy <= cyHi; ++cy) {
            const world::ChunkPos pos{col.cx, cy, col.cz};
            bool caves = shallowCaves && world::caveAir(tp_, pos, hm, air, cut);
            {   // Р88: руда этого куска. Берётся ЗДЕСЬ, пока маска пустоты не правлена под LOD:
                // руда сидит в камне и в полостях висеть не должна.
                const int32_t bx0 = col.cx * world::CHUNK, bz0 = col.cz * world::CHUNK, by0 = cy * world::CHUNK;
                world::oreSpecks(tp_.seed, bx0, by0, bz0, world::CHUNK, specks);
                for (const world::OreSpeck& sp : specks) {
                    const int32_t bx = fd(sp.x, world::FINE), by = fd(sp.y, world::FINE), bz = fd(sp.z, world::FINE);
                    // Хозяин вкрапления - свой столбец и свой кусок: иначе одно и то же тело выложат
                    // оба соседа, и грани встанут друг в друга.
                    if (bx < bx0 || bx >= bx0 + world::CHUNK || bz < bz0 || bz >= bz0 + world::CHUNK) continue;
                    if (by < by0 || by >= by0 + world::CHUNK) continue;
                    const int lx = bx - bx0, lz = bz - bz0;
                    if (caves && ((air[lz + 1][lx + 1] >> (by - by0 + 1)) & 1)) continue;   // здесь пустота
                    const int32_t vcut = cut[lz + 1][lx + 1];
                    const int32_t top = hm.h[lz + 1][lx + 1] - vcut;
                    world::Cover cv = hm.cover[lz][lx];
                    cv.soil = int16_t(std::max(0, int(cv.soil) - vcut));
                    const world::Material host =
                        world::columnMaterial(top, cv, by, hm.water[lz + 1][lx + 1], hm.rock, bx, bz);
                    if (host < world::ROCK_FIRST || host > world::ROCK_LAST) continue;   // руда сидит в камне
                    const uint8_t ore = uint8_t(world::pickOre(host, top - by, sp.roll));
                    const int32_t mnx = sp.x - sp.a - 1, mny = sp.y - sp.c - 1, mnz = sp.z - sp.a - 1;
                    const int32_t c[3] = {fd(mnx, 40), fd(mny, 40), fd(mnz, 40)};
                    const uint64_t key = uint64_t(uint32_t(c[0])) << 42 ^ uint64_t(uint32_t(c[1]) & 0x3FFFFF) << 21 ^
                                         uint64_t(uint32_t(c[2]) & 0x1FFFFF);
                    cells[key].push_back({sp, ore});
                }
            }
            if (caves && caveFloor != INT32_MIN) {   // Р39: ниже пола (с соседями) пещеры закрыты; заглушки у соседей без пещер
                const int64_t n = std::clamp<int64_t>(int64_t(caveFloor) - (int64_t(cy) * world::CHUNK - 1), 0, 34);
                const uint64_t keep = n >= 64 ? 0 : ~((uint64_t(1) << n) - 1);
                constexpr int E = world::CHUNK + 1;
                bool left = false;
                for (int z = 0; z <= E; ++z)
                    for (int x = 0; x <= E; ++x) {
                        uint64_t& a = air[z][x];
                        a &= keep;
                        if (((shallowCaps & CAP_NX) && x <= 1) || ((shallowCaps & CAP_PX) && x >= E - 1) ||
                            ((shallowCaps & CAP_NZ) && z <= 1) || ((shallowCaps & CAP_PZ) && z >= E - 1))
                            a = 0;
                        left |= a != 0;
                    }
                caves = left;
            }
            const bool underground = cy < cyLo;
            if (caves) {
                // Р27, заглушки: ниже самого нижнего построенного куска ничего нет - его нижний слой и кайма под
                // ним - камень; у подземных кусков на краю подземного яруса - так же крайний слой и кайма сбоку
                constexpr int E = world::CHUNK + 1;   // индекс каймы за дальним краем
                bool left = false;
                for (int z = 0; z <= E; ++z)
                    for (int x = 0; x <= E; ++x) {
                        uint64_t& a = air[z][x];
                        if (cy == cyFrom) a &= ~uint64_t(3);
                        if (underground && (((col.caps & CAP_NX) && x <= 1) || ((col.caps & CAP_PX) && x >= E - 1) ||
                                            ((col.caps & CAP_NZ) && z <= 1) || ((col.caps & CAP_PZ) && z >= E - 1)))
                            a = 0;
                        left |= a != 0;
                    }
                caves = left;
            }
            // Р83: кусок с правкой строим всегда - в нём есть пустота, которой нет в формуле
            const bool edited = editLo != INT32_MAX && cy >= editLo && cy <= editHi;
            if (!caves && underground && !edited) continue;   // подземный и без пещер - сплошной камень
            // нетронутый кусок: сетка прямо от высот (и пещер), без заполнения вокселей (Р22)
            world::MeshStats ms;
            world::buildMesh(pos, hm, verts, ms, caves ? air : nullptr, cut, edits);
            if (verts.empty()) continue;
            r.chunks.push_back({{col.cx * world::CHUNK, cy * world::CHUNK, col.cz * world::CHUNK}, verts, ms.quads, ms.waterQuads, underground});
        }
        // ─── Р88: РУДА - отдельный слой мелких сеток (собрана выше, по кускам)
        {
            for (auto& [key, list] : cells) {
                const auto fd = [](int32_t a, int32_t b) { return a >= 0 ? a / b : -((-a + b - 1) / b); };
                const world::OreSpeck& f0 = list[0].first;
                const int32_t c0 = fd(f0.x - f0.a - 1, 40) * 40, c1 = fd(f0.y - f0.c - 1, 40) * 40,
                              c2 = fd(f0.z - f0.a - 1, 40) * 40;
                // основа сетки - блок, а остаток до начала ячейки уходит в сдвиг мелких вокселей
                const int32_t b0 = fd(c0, 16), b1 = fd(c1, 16), b2 = fd(c2, 16);
                verts.clear();
                int quads = 0;
                for (const auto& [sp, ore] : list) {
                    const int lx = int(sp.x - c0), ly = int(sp.y - c1), lz = int(sp.z - c2);
                    addSpeck(verts, lx, ly, lz, sp, ore, quads);
                }
                if (verts.empty()) continue;
                r.chunks.push_back({{b0, b1, b2}, verts, quads, 0, false, true,
                                    {c0 - b0 * 16, c1 - b1 * 16, c2 - b2 * 16}});
            }
        }
        r.ms = msSince(t0);
        std::lock_guard lk(mu_);
        buildingColumns_.erase(colKey(col.cx, col.cz));
        doneColumns_.push_back(std::move(r));
    }
}
