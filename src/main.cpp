// Отмель: точка входа. Окно, мир из зерна (карта высот, биомы - Р19): вблизи воксели, вдали гладкие плитки
// CDLOD (Р20); летающая камера.
// В заголовке окна - замеры (Р4): время кадра, генерации и сборки сеток, число кусков и прямоугольников.
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <format>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>

#include "frustum.h"
#include "journal.h"
#include "render/render.h"
#include "world/flow.h"
#include "world/lod.h"
#include "world/mesher.h"
#include "world/oak.h"
#include "world/water_sim.h"
#include "world/terrain.h"
#include "builder.h"
#include "voxel_zone.h"

namespace {

using clock_t_ = std::chrono::steady_clock;
double msSince(clock_t_::time_point t) { return std::chrono::duration<double, std::milli>(clock_t_::now() - t).count(); }

// Дальний рельеф (Р20): плитки CDLOD. Каждый кадр - выбор по расстоянию, отсечение по пирамиде видимости.
// Недостающие плитки заказываются общему пулу (Р21); пока плитки нет, её место закрывает та же четверть
// плитки-родителя - грубее, но без дыры. Лишние плитки вытесняются.
class FarTerrain {
public:
    struct Stats { int built = 0; double workMs = 0; size_t cached = 0; int substituted = 0; };

    FarTerrain(render::Backend& gpu, Builder& builder) : gpu_(gpu), builder_(builder) {}
    ~FarTerrain() { for (auto& [k, e] : cache_) gpu_.destroyMesh(e.mesh); }

    void frame(const world::LodSettings& s, glm::vec3 eye, const Frustum& fr, const VoxelZone& voxels,
               render::CommandList& out, Stats& st) {
        ++frame_;
        // готовые плитки - в видеопамять (дёшево: 13 КБ на плитку)
        Builder::TileResult r;
        for (int n = 0; n < 512 && builder_.popTile(r); ++n) {
            Entry e;
            e.mesh = gpu_.uploadTile(r.mesh.data, r.mesh.water);
            e.minY = float(r.mesh.minH) / world::VOXELS_PER_BLOCK;
            e.maxY = float(r.mesh.maxH + 1) / world::VOXELS_PER_BLOCK;
            e.used = frame_;
            if (Entry* old = find(r.key)) gpu_.destroyMesh(old->mesh);
            cache_[world::tileHash(r.key)] = e;
            ++st.built;
            st.workMs += r.ms;
        }

        world::selectTiles(s, eye.x, eye.z, sel_);
        missing_.clear();
        st.substituted = 0;
        for (world::TileDraw d : sel_) {
            const int64_t size = world::tileBlocks(d.key.level), half = size / 2;
            const int64_t x0 = int64_t(d.key.tx) * size, z0 = int64_t(d.key.tz) * size;
            Entry* e = find(d.key);
            // отсечение по четвертям; высота коробки - из готовой плитки, у неготовой - весь диапазон мира (Р10)
            const float lo = e ? e->minY : -500.0f, hi = e ? e->maxY : 1000.0f;
            const uint8_t wanted = d.quadrants;
            for (int q = 0; q < 4; ++q)
                if (d.quadrants >> q & 1) {
                    const int64_t qx = x0 + (q & 1) * half, qz = z0 + (q >> 1) * half;
                    // целиком над готовыми вокселями (Р21) или вне взгляда - не рисуем
                    if (voxels.covered(qx, qz, half) ||
                        !fr.visible({float(qx), lo, float(qz)}, {float(qx + half), hi, float(qz + half)}))
                        d.quadrants &= uint8_t(~(1 << q));
                }
            if (!d.quadrants) continue;
            if (e) { emit(s, *e, d.key, d.quadrants, out); continue; }
            missing_.push_back(d.key);
            // подмена: плитка целиком (все 4 четверти её) = одна четверть родителя - рисуем её, если родитель готов
            const world::TileKey parent{d.key.level + 1, d.key.tx >> 1, d.key.tz >> 1};
            Entry* p = wanted == 0xF ? find(parent) : nullptr;
            if (p) {
                emit(s, *p, parent, uint8_t(1 << ((d.key.tx & 1) | (d.key.tz & 1) << 1)), out);
                ++st.substituted;
            }
        }
        builder_.requestTiles(missing_);
        evict();
        st.cached = cache_.size();
    }

private:
    struct Entry { render::MeshId mesh = render::NO_MESH; float minY = 0, maxY = 0; uint64_t used = 0; };

    Entry* find(world::TileKey k) {
        auto it = cache_.find(world::tileHash(k));
        return it == cache_.end() ? nullptr : &it->second;
    }
    void emit(const world::LodSettings& s, Entry& e, world::TileKey k, uint8_t quadrants, render::CommandList& out) {
        e.used = frame_;
        const int L = k.level;
        const float end = float(world::lodRange(s, L));
        // морфинг - на последних 30% промежутка между дальностями уровней L-1 и L
        const float start = end - 0.3f * (end - end / 2);
        out.drawTile({e.mesh, {int(int64_t(k.tx) * world::tileBlocks(L)), int(int64_t(k.tz) * world::tileBlocks(L))},
                      1 << L, L, start, end, quadrants});
    }

    // вытеснение: держим не больше LIMIT плиток, выбрасываем давно не нужные
    void evict() {
        constexpr size_t LIMIT = 6000;
        if (cache_.size() <= LIMIT) return;
        std::vector<std::pair<uint64_t, uint64_t>> byAge;   // (кадр последнего использования, ключ)
        for (auto& [k, e] : cache_) if (e.used + 60 < frame_) byAge.push_back({e.used, k});
        std::sort(byAge.begin(), byAge.end());
        for (size_t i = 0; i < byAge.size() && cache_.size() > LIMIT; ++i) {
            gpu_.destroyMesh(cache_[byAge[i].second].mesh);
            cache_.erase(byAge[i].second);
        }
    }

    render::Backend& gpu_;
    Builder& builder_;
    std::unordered_map<uint64_t, Entry> cache_;
    std::vector<world::TileDraw> sel_;
    std::vector<world::TileKey> missing_;
    uint64_t frame_ = 0;
};

}  // namespace

// Снимок: кадр в BMP (строки переворачиваем - видеокарта отдаёт снизу вверх)
void saveShot(render::Backend& gpu, int w, int h, const char* path) {
    std::vector<uint8_t> rgb;
    gpu.readPixels(w, h, rgb);
    SDL_Surface* s = SDL_CreateSurface(w, h, SDL_PIXELFORMAT_RGB24);
    for (int y = 0; y < h; ++y)
        std::memcpy(static_cast<uint8_t*>(s->pixels) + y * s->pitch, &rgb[static_cast<size_t>(h - 1 - y) * w * 3], static_cast<size_t>(w) * 3);
    SDL_SaveBMP(s, path);
    SDL_DestroySurface(s);
    journal::info("окно", "снимок: {}", path);
}

int main(int argc, char** argv) {
    // Ключи (пока вместо меню настроек):
    //   --voxels B   зона вокселей: квадрат ±B блоков вокруг точки старта (по умолчанию 128) - дальше гладко (Р20)
    //   --draw B     дальность видимости, блоков (24576)
    //   --detail K   детальность дальних уровней: дальность уровня 0, блоков (128; не меньше 65 - иначе щели)
    //   --at X Z     точка старта, блоки;  --eye x y z yaw pitch - камера;  --shot - снимок и выход
    //   --fly V      (со --shot) облёт: камера летит вперёд со скоростью V блоков/с 10 секунд, замер отставания
    bool shot = false;
    float recordSec = 0;   // со --shot: запись кадров rec_NNN.bmp 15 раз в секунду столько секунд
    int voxelBlocks = 128;
    float atX = 0, atZ = 0, flySpeed = 0;
    world::LodSettings lod;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--shot") shot = true;
        if (a == "--voxels" && i + 1 < argc) voxelBlocks = std::max(2, std::stoi(argv[i + 1]));
        if (a == "--draw" && i + 1 < argc) lod.drawDistance = std::max(256, std::stoi(argv[i + 1]));
        if (a == "--detail" && i + 1 < argc) lod.detail = std::max(65, std::stoi(argv[i + 1]));
        if (a == "--at" && i + 2 < argc) { atX = std::stof(argv[i + 1]); atZ = std::stof(argv[i + 2]); }
        if (a == "--fly" && i + 1 < argc) flySpeed = std::stof(argv[i + 1]);
        if (a == "--record" && i + 1 < argc) recordSec = std::stof(argv[i + 1]);
    }
    if (!SDL_Init(SDL_INIT_VIDEO)) { journal::error("окно", "SDL не запустился: {}", SDL_GetError()); return 1; }
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 6);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG);
    // глубина и сглаживание - в своём буфере кадра (Р20), окну нужен только цвет
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 0);

    int width = 1280, height = 720;
    SDL_Window* window = SDL_CreateWindow("Отмель", width, height, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (!window) { journal::error("окно", "окно не создано: {}", SDL_GetError()); return 1; }

    const std::string shaderDir = std::string(SDL_GetBasePath()) + "shaders";
    auto gpu = render::createGL(window, shaderDir.c_str());
    if (!gpu) return 1;

    // Зона вокселей едет за камерой (Р21): ±R столбцов кусков (столбец - 2×2 блока); строят фоновые потоки
    const int blocksPerColumn = world::CHUNK / world::VOXELS_PER_BLOCK;
    world::TerrainParams tp;
    for (int i = 1; i + 1 < argc; ++i)
        if (std::string(argv[i]) == "--style") tp.mountainStyle = std::stoi(argv[i + 1]);   // вид гор (Р23)
    Builder builder(tp);
    VoxelZone voxels(*gpu, builder, std::max(1, voxelBlocks / blocksPerColumn));
    FarTerrain farTerrain(*gpu, builder);

    // Камера: в блоках. Мышь - взгляд, WASD - движение, пробел/Ctrl - вверх/вниз, Shift - быстрее,
    // колесо - скорость, L - раскраска дальних уровней, Esc - выход. Старт - в 48 блоках мира над землёй (Р54: 3 прежних).
    const float ground = float(world::reliefAt(tp, int32_t(atX * 16), int32_t(atZ * 16)).height) / world::VOXELS_PER_BLOCK;
    glm::vec3 eye(atX, ground + 3.0f, atZ);
    float yaw = -0.6f, pitch = -0.35f, speedScale = 1.0f;
    int debug = 0;
    for (int i = 1; i + 5 < argc; ++i)
        if (std::string(argv[i]) == "--eye") {
            eye = {std::stof(argv[i + 1]), std::stof(argv[i + 2]), std::stof(argv[i + 3])};
            yaw = std::stof(argv[i + 4]);
            pitch = std::stof(argv[i + 5]);
        }
    // Р53: дубы-коряги для просмотра: левая кнопка - новый дуб в 32 блоках перед камерой, правая - убрать все,
    // R - корни под землёй прятать / показывать. Сетка - кусками 32³ поверх мира (в мир пока не вписаны).
    struct OakSpot { int32_t x, y, z; uint32_t seed; };
    struct OakMesh { render::MeshId mesh; glm::ivec3 base, fineOff; };
    std::vector<OakSpot> oaks;
    std::vector<OakMesh> oakMeshes;
    std::vector<std::pair<render::MeshId, glm::ivec3>> rulerMeshes;
    bool hideRoots = true;
    uint32_t oakSeed = 1;
    for (int i = 1; i + 3 < argc; ++i)
        if (std::string(argv[i]) == "--oak") {
            const int32_t x = int32_t(std::floor(std::stof(argv[i + 1]) * 16)), z = int32_t(std::floor(std::stof(argv[i + 2]) * 16));
            oaks.push_back({x, 0, z, uint32_t(std::stoul(argv[i + 3]))});
        }
    // основание - на самой низкой земле под комлем (не висит над склоном), чуть утоплено
    const auto oakBase = [&](int32_t x, int32_t z) {
        int32_t g = INT32_MAX;
        for (const auto [dx, dz] : {std::pair{0, 0}, {2, 0}, {-2, 0}, {0, 2}, {0, -2}})
            g = std::min(g, world::reliefAt(tp, x + dx, z + dz).height);
        return g + 1;
    };
    for (OakSpot& o : oaks) o.y = oakBase(o.x, o.z);
    // эталон размеров (--ruler X Z): столб из 10 блоков мира (Р54: воксель = блок), чередуются по цвету
    glm::ivec3 ruler(INT32_MIN);
    for (int i = 1; i + 2 < argc; ++i)
        if (std::string(argv[i]) == "--ruler") {
            ruler.x = int32_t(std::floor(std::stof(argv[i + 1]) * 16));
            ruler.z = int32_t(std::floor(std::stof(argv[i + 2]) * 16));
            ruler.y = world::reliefAt(tp, ruler.x, ruler.z).height + 1;
        }
    // Р56: дуб - из мелких вокселей (1/16 блока): основа - блок над землёй под комлем, куски 32³ мелких вокселей
    // (2×2×2 блока) - от основы; корни прячутся, где мелкий воксель внутри блока земли
    const auto buildOaks = [&] {
        for (const OakMesh& m : oakMeshes) gpu->destroyMesh(m.mesh);
        oakMeshes.clear();
        static world::ColumnHeights air;
        for (auto& row : air.h) for (int32_t& v : row) v = INT32_MIN / 4;
        for (auto& row : air.water) for (int32_t& v : row) v = world::NO_WATER;
        const auto fdiv = [](int32_t v, int32_t d) { return v >= 0 ? v / d : -((-v + d - 1) / d); };
        std::unordered_map<int64_t, int32_t> groundOf;   // верх земли блока-столбца (кэш)
        const auto groundAt = [&](int32_t bx, int32_t bz) {
            const int64_t key = (int64_t(bx) << 32) ^ uint32_t(bz);
            if (auto it = groundOf.find(key); it != groundOf.end()) return it->second;
            world::ColumnHeights hm;
            const int32_t cx = fdiv(bx, world::CHUNK), cz = fdiv(bz, world::CHUNK);
            world::columnHeights(tp, cx, cz, hm);
            for (int z = 0; z < world::CHUNK; ++z)
                for (int x = 0; x < world::CHUNK; ++x)
                    groundOf[(int64_t(cx * world::CHUNK + x) << 32) ^ uint32_t(cz * world::CHUNK + z)] = hm.h[z + 1][x + 1];
            return groundOf[key];
        };
        constexpr int SINK = 3;   // комель утоплен на 3 мелких вокселя
        for (const OakSpot& o : oaks) {
            const auto t0 = clock_t_::now();
            world::OakVoxels ov;
            world::growOak(o.seed, ov);
            const glm::ivec3 base(o.x, o.y, o.z);   // блок-основа: верх земли + 1
            int quads = 0, hidden = 0;
            auto c = std::make_unique<world::Chunk>();
            std::vector<uint32_t> verts;
            // мелкие воксели от основы: f = local + (x0, y0 - SINK, z0)
            const int fx0 = ov.x0, fy0 = ov.y0 - SINK, fz0 = ov.z0;
            for (int cz = fdiv(fz0, 32); cz <= fdiv(fz0 + ov.sz - 1, 32); ++cz)
                for (int cx = fdiv(fx0, 32); cx <= fdiv(fx0 + ov.sx - 1, 32); ++cx)
                    for (int cy = fdiv(fy0, 32); cy <= fdiv(fy0 + ov.sy - 1, 32); ++cy) {
                        c->pos = {cx, cy, cz};
                        c->edited = true;
                        c->voxels.fill(world::AIR);
                        bool any = false;
                        for (int y = 0; y < 32; ++y)
                            for (int z = 0; z < 32; ++z)
                                for (int x = 0; x < 32; ++x) {
                                    const int fx = cx * 32 + x, fy = cy * 32 + y, fz = cz * 32 + z;
                                    const int lx = fx - fx0, ly = fy - fy0, lz = fz - fz0;
                                    if (lx < 0 || ly < 0 || lz < 0 || lx >= ov.sx || ly >= ov.sy || lz >= ov.sz) continue;
                                    const size_t k = (size_t(ly) * ov.sz + lz) * ov.sx + lx;
                                    const world::Material m = ov.v[k];
                                    if (m == world::AIR) continue;
                                    if (hideRoots && ov.root[k]) {
                                        // мелкий воксель - в блоке (bx, by, bz); под землёй - не рисуем
                                        const int32_t bx = base.x + fdiv(fx, 16), by = base.y + fdiv(fy, 16), bz = base.z + fdiv(fz, 16);
                                        if (by <= groundAt(bx, bz)) { ++hidden; continue; }
                                    }
                                    c->at(x, y, z) = m;
                                    any = true;
                                }
                        if (!any) continue;
                        world::MeshStats ms;
                        world::buildMesh(*c, air, verts, ms);
                        if (verts.empty()) continue;
                        quads += ms.quads;
                        oakMeshes.push_back({gpu->uploadChunkMesh(verts, 0), base, glm::ivec3(cx, cy, cz) * 32});
                    }
            journal::info("дуб", "семя {}: узлов {}, мелких вокселей древесины {} (корней {}, спрятано под землёй {}), листвы {}; высота над землёй {:.1f} блока; прямоугольников {}; {:.0f} мс",
                          o.seed, ov.nodes, ov.wood, ov.roots, hidden, ov.leaves, (ov.top - SINK) / 16.0, quads, msSince(t0));
        }
    };
    if (!oaks.empty()) buildOaks();
    if (ruler.x != INT32_MIN) {
        static world::ColumnHeights air;
        for (auto& row : air.h) for (int32_t& v : row) v = INT32_MIN / 4;
        for (auto& row : air.water) for (int32_t& v : row) v = world::NO_WATER;
        const auto fdiv = [](int32_t v) { return v >= 0 ? v / world::CHUNK : -((-v + world::CHUNK - 1) / world::CHUNK); };
        const auto solid = [&](int32_t x, int32_t y, int32_t z) -> world::Material {
            const int32_t lx = x - ruler.x, ly = y - ruler.y, lz = z - ruler.z;
            if (lx == 0 && lz == 0 && ly >= 0 && ly < 10) return (ly & 1) ? world::SNOW : world::BARK;   // 10 блоков
            return world::AIR;
        };
        auto c = std::make_unique<world::Chunk>();
        std::vector<uint32_t> verts;
        for (int32_t cy = fdiv(ruler.y); cy <= fdiv(ruler.y + 17); ++cy)
            for (int32_t cz = fdiv(ruler.z); cz <= fdiv(ruler.z + 16); ++cz)
                for (int32_t cx = fdiv(ruler.x); cx <= fdiv(ruler.x + 21); ++cx) {
                    c->pos = {cx, cy, cz};
                    c->edited = true;
                    bool any = false;
                    for (int y = 0; y < world::CHUNK; ++y)
                        for (int z = 0; z < world::CHUNK; ++z)
                            for (int x = 0; x < world::CHUNK; ++x) {
                                const world::Material m = solid(cx * 32 + x, cy * 32 + y, cz * 32 + z);
                                c->at(x, y, z) = m;
                                any |= m != world::AIR;
                            }
                    if (!any) continue;
                    world::MeshStats ms;
                    world::buildMesh(*c, air, verts, ms);
                    if (!verts.empty()) rulerMeshes.push_back({gpu->uploadChunkMesh(verts, 0), glm::ivec3(cx, cy, cz) * world::CHUNK});
                }
        journal::info("эталон", "куб в блок у ({}, {}, {}) воксели", ruler.x, ruler.y, ruler.z);
    }
    if (!shot) SDL_SetWindowRelativeMouseMode(window, true);
    // в режиме снимка - без ожидания обновления экрана: замер времени кадра не упирается в 60 Гц
    if (shot) SDL_GL_SetSwapInterval(0);
    int frameNo = 0;
    double shotMs = 0, shotMaxMs = 0;
    int maxPending = 0;
    bool warm = false;                       // снимок: зона вокселей достроена
    double part[3] = {};                     // худшее за кадр: зона вокселей, дальние плитки, отрисовка
    auto shotStart = clock_t_::now();

    const render::Color sky{0.55f, 0.72f, 0.88f, 1.0f};
    render::CommandList frame;
    render::FrameStats fs;
    FarTerrain::Stats farStats;
    auto last = clock_t_::now(), titleAt = last;
    const auto startAt = last;
    // Р43: поле течений вокруг камеры - в фоне; пересчёт, когда камера ушла от центра поля на четверть
    constexpr int FLOW_SIZE = 256;
    struct FlowData { std::vector<float> vel, ground, surface; };
    std::future<FlowData> flowJob;
    glm::ivec2 flowAt(INT32_MIN / 2), flowPending(0);
    glm::vec4 flowArea(0.0f);
    // Р43, шаг 3: автомат воды в середине поля (128×128 блоков у камеры): течёт сам, на ступенях и обрывах
    // разгоняется (водопады); его скорости раз в 4 кадра заменяют поле течений в своей зоне
    constexpr int SIM = 128, SIM_OFF = (FLOW_SIZE - SIM) / 2;
    world::WaterSim sim;
    sim.holdEdges = true;
    sim.keepChannel = true;
    bool simOn = false;
    std::vector<float> fieldVel, shownVel, simVerts;
    // Р52: брызги и капли - воксели у камеры: рождаются на быстрой воде (скаты, пороги, водопады), летят по
    // течению и вверх, падают и пропадают
    struct Drop { glm::vec3 p, v; float life; };
    std::vector<Drop> drops;
    std::vector<float> flowGround, flowSurface, dropVerts;
    // Р58: мелкая вода у камеры - зона 128×128 блоков (4×4 куска); уровни по блокам - в фоне при смене куска камеры
    constexpr int FINE = 128;
    constexpr float NONE = -1e9f;
    struct FineData { glm::ivec2 origin; std::vector<float> vb, ground; std::vector<uint8_t> wet; float flat[FINE * FINE]; };
    std::future<std::unique_ptr<FineData>> fineJob;
    glm::ivec2 fineAt(INT32_MIN / 2);
    std::unique_ptr<FineData> fineData;
    uint32_t dropRng = 0x9E3779B9u;
    const auto rnd = [&dropRng] { dropRng ^= dropRng << 13; dropRng ^= dropRng >> 17; dropRng ^= dropRng << 5; return float(dropRng >> 8) / 16777216.0f; };
    int simFrame = 0;
    glm::vec4 simArea(0.0f);
    double sumMs = 0;
    int frames = 0, chunksDrawn = 0;
    bool running = true;
    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_EVENT_QUIT) running = false;
            if (ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_ESCAPE) running = false;
            if (ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_L) gpu->setDebug(debug ^= 1);
            if (ev.type == SDL_EVENT_MOUSE_BUTTON_DOWN && ev.button.button == SDL_BUTTON_LEFT) {
                const glm::vec2 f = glm::normalize(glm::vec2(std::cos(yaw), std::sin(yaw)));
                const int32_t x = int32_t(std::floor((eye.x + f.x * 2.0f) * 16)), z = int32_t(std::floor((eye.z + f.y * 2.0f) * 16));   // 32 блока
                oakSeed = oakSeed * 1664525u + 1013904223u;
                oaks.push_back({x, oakBase(x, z), z, oakSeed});
                buildOaks();
            }
            if (ev.type == SDL_EVENT_MOUSE_BUTTON_DOWN && ev.button.button == SDL_BUTTON_RIGHT) { oaks.clear(); buildOaks(); }
            if (ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_R) { hideRoots = !hideRoots; buildOaks(); }
            if (ev.type == SDL_EVENT_MOUSE_WHEEL) speedScale = std::clamp(speedScale * std::pow(1.25f, ev.wheel.y), 0.1f, 100.0f);
            if (ev.type == SDL_EVENT_MOUSE_MOTION) {
                yaw += ev.motion.xrel * 0.0025f;
                pitch = std::clamp(pitch - ev.motion.yrel * 0.0025f, -1.55f, 1.55f);
            }
            if (ev.type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED) {
                width = ev.window.data1; height = ev.window.data2;
                gpu->resize(width, height);
            }
        }
        const auto now = clock_t_::now();
        const float dt = std::chrono::duration<float>(now - last).count();
        sumMs += std::chrono::duration<double, std::milli>(now - last).count();
        ++frames;
        last = now;

        const glm::vec3 fwd(std::cos(pitch) * std::cos(yaw), std::sin(pitch), std::cos(pitch) * std::sin(yaw));
        const glm::vec3 right = glm::normalize(glm::cross(fwd, glm::vec3(0, 1, 0)));
        const bool* keys = SDL_GetKeyboardState(nullptr);
        // Р56: камера - в прежних мерах (16 блоков мира): шаг ~5 блоков/с, с Shift ~48 - чтобы разглядеть мелочь
        const float speed = (keys[SDL_SCANCODE_LSHIFT] ? 3.0f : 0.3f) * speedScale * dt;
        if (keys[SDL_SCANCODE_W]) eye += fwd * speed;
        if (keys[SDL_SCANCODE_S]) eye -= fwd * speed;
        if (keys[SDL_SCANCODE_D]) eye += right * speed;
        if (keys[SDL_SCANCODE_A]) eye -= right * speed;
        if (keys[SDL_SCANCODE_SPACE]) eye.y += speed;
        if (keys[SDL_SCANCODE_LCTRL]) eye.y -= speed;
        if (shot && warm && flySpeed > 0) eye += glm::normalize(glm::vec3(fwd.x, 0, fwd.z)) * flySpeed * dt;

        // обратная глубина (Р20): ближняя и дальняя плоскости переставлены местами
        const float farPlane = float(lod.drawDistance) * 1.5f;
        const glm::mat4 proj = glm::perspectiveRH_ZO(glm::radians(70.0f), float(width) / float(std::max(1, height)), farPlane, 0.002f);   // Р56: ближняя плоскость - 3 см (вплотную к коре и листьям)
        const glm::mat4 viewProj = proj * glm::lookAt(eye, eye + fwd, glm::vec3(0, 1, 0));
        const Frustum fr(viewProj);
        const auto tUpd = clock_t_::now();
        voxels.update(eye, fr, 4.0);   // выгрузка готовых столбцов - не дольше 4 мс за кадр
        part[0] = std::max(part[0], msSince(tUpd));

        {
            const glm::ivec2 want((int(std::floor(eye.x)) - FLOW_SIZE / 2) & ~31, (int(std::floor(eye.z)) - FLOW_SIZE / 2) & ~31);
            if (!flowJob.valid() && (std::abs(want.x - flowAt.x) > FLOW_SIZE / 4 || std::abs(want.y - flowAt.y) > FLOW_SIZE / 4)) {
                flowPending = want;
                flowJob = std::async(std::launch::async, [&tp, want] {
                    FlowData f;
                    world::flowField(tp, want.x, want.y, FLOW_SIZE, f.vel, &f.ground, &f.surface);
                    return f;
                });
            }
            if (flowJob.valid() && flowJob.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                FlowData f = flowJob.get();
                fieldVel = std::move(f.vel);
                flowGround = f.ground;
                flowSurface = f.surface;
                gpu->updateFlowField(FLOW_SIZE, fieldVel);
                flowAt = flowPending;
                flowArea = glm::vec4(float(flowAt.x), float(flowAt.y), float(FLOW_SIZE), 0.0f);
                std::vector<float> g(SIM * SIM), s(SIM * SIM);
                for (int z = 0; z < SIM; ++z)
                    for (int x = 0; x < SIM; ++x) {
                        g[size_t(z) * SIM + x] = f.ground[size_t(z + SIM_OFF) * FLOW_SIZE + x + SIM_OFF];
                        s[size_t(z) * SIM + x] = f.surface[size_t(z + SIM_OFF) * FLOW_SIZE + x + SIM_OFF];
                    }
                sim.init(SIM, SIM, g, s);
                simOn = true;
            }
            if (simOn) {
                const float h = std::min(dt, 1.0f / 30.0f) * 0.5f;
                sim.step(h);
                sim.step(h);
                if (++simFrame % 4 == 0) {
                    shownVel = fieldVel;
                    for (int z = 0; z < SIM; ++z)
                        for (int x = 0; x < SIM; ++x) {
                            if (sim.depth(x, z) < 0.05f) continue;
                            float vx, vz;
                            sim.velocity(x, z, vx, vz);
                            const size_t k = (size_t(z + SIM_OFF) * FLOW_SIZE + x + SIM_OFF) * 2;
                            shownVel[k] = vx;
                            shownVel[k + 1] = vz;
                        }
                    gpu->updateFlowField(FLOW_SIZE, shownVel);
                }
            }
        }
        // Р52: брызги - у камеры, где вода быстрее 2.5 м/с; на перепадах (вода у соседа ниже по течению ниже) -
        // вчетверо чаще. Р55 (Р54: воксель = блок мира): поле и камера - в прежних мерах (16 блоков мира);
        // капля - 2 текселя (1/8 блока), летит по течению, вбок и вверх (1-3 м/с), тяжесть 9.8 м/с², живёт
        // 0.3..0.7 с, падая в воду - пропадает; рождаются густо в 96 блоках вокруг камеры
        if (!flowGround.empty()) {
            const std::vector<float>& vel = shownVel.empty() ? fieldVel : shownVel;
            const float hdt = std::min(dt, 0.05f);
            const int ex = int(std::floor(eye.x)) - flowAt.x, ez = int(std::floor(eye.z)) - flowAt.y;
            constexpr int R = 6;
            constexpr float U = 1.0f / 16.0f;   // блок мира в прежних мерах
            for (int z = std::max(0, ez - R); z < std::min(FLOW_SIZE, ez + R); ++z)
                for (int x = std::max(0, ex - R); x < std::min(FLOW_SIZE, ex + R); ++x) {
                    const size_t k = size_t(z) * FLOW_SIZE + x;
                    const float sf = flowSurface[k];
                    if (sf <= flowGround[k] + 0.05f) continue;
                    const float vx = vel[k * 2], vz = vel[k * 2 + 1], sp = std::sqrt(vx * vx + vz * vz);
                    if (sp < 2.5f) continue;
                    float rate = (sp - 2.5f) * 80.0f;
                    const int dx = x + int(std::round(vx / sp)), dz = z + int(std::round(vz / sp));
                    if (dx >= 0 && dz >= 0 && dx < FLOW_SIZE && dz < FLOW_SIZE && flowSurface[size_t(dz) * FLOW_SIZE + dx] < sf - 0.5f) rate *= 4.0f;
                    for (float k = rate * hdt; k > 0 && drops.size() < 8000; k -= 1.0f) {
                        if (k < 1.0f && rnd() >= k) break;
                        const float up = (1.0f + 2.0f * rnd()) * std::min(1.0f, sp / 6.0f);
                        drops.push_back({glm::vec3(float(flowAt.x + x) + rnd(), sf + 0.05f * U, float(flowAt.y + z) + rnd()),
                                         glm::vec3(vx * 0.35f + (rnd() - 0.5f) * 1.2f, up, vz * 0.35f + (rnd() - 0.5f) * 1.2f) * U,
                                         0.3f + 0.4f * rnd()});
                    }
                }
            for (Drop& d : drops) {
                d.v.y -= 9.8f * U * hdt;
                d.p += d.v * hdt;
                d.life -= hdt;
                const int bx = int(std::floor(d.p.x)) - flowAt.x, bz = int(std::floor(d.p.z)) - flowAt.y;
                if (d.v.y < 0 && bx >= 0 && bz >= 0 && bx < FLOW_SIZE && bz < FLOW_SIZE &&
                    d.p.y < std::max(flowSurface[size_t(bz) * FLOW_SIZE + bx], flowGround[size_t(bz) * FLOW_SIZE + bx]))
                    d.life = 0;
            }
            std::erase_if(drops, [](const Drop& d) { return d.life <= 0; });
            // капля - кубик в 2 текселя, три грани к камере (нормаль + 6: шейдер рисует её брызгами)
            dropVerts.clear();
            constexpr float S = 2.0f / 256.0f;
            for (const Drop& d : drops) {
                const glm::vec3 b = glm::floor(d.p * 256.0f) / 256.0f;   // по сетке текселей
                for (int ax = 0; ax < 3; ++ax) {
                    const bool plus = eye[ax] > b[ax] + S * 0.5f;
                    const float n = float(ax * 2 + (plus ? 0 : 1) + 6);
                    const int u = (ax + 1) % 3, w = (ax + 2) % 3;
                    glm::vec3 c[4];
                    for (int q = 0; q < 4; ++q) {
                        c[q] = b;
                        c[q][ax] += plus ? S : 0.0f;
                        c[q][u] += (q == 1 || q == 2) ? S : 0.0f;
                        c[q][w] += (q >= 2) ? S : 0.0f;
                    }
                    for (int q : {0, 1, 2, 0, 2, 3}) dropVerts.insert(dropVerts.end(), {c[q].x, c[q].y, c[q].z, n});
                }
            }
            gpu->setSimWater(dropVerts);
        }
        // Р58: мелкая вода. Зона - от куска камеры (32 блока) на 2 куска в стороны; данные (+2 блока каймы для
        // соседей): vb - верх глади в центре блока (у мокрого - верх воды; у сухого рядом с водой - вровень с самой
        // низкой соседней водой или его земля, если ниже), верх земли. Блок рисуется мелкой водой, если он мокрый и
        // гладь в 3×3 вокруг не ровная (ступень, скат) или вода быстрее 1.5 м/с; там вода кусков прячется.
        {
            const glm::ivec2 eb(int(std::floor(eye.x * 16.0f)), int(std::floor(eye.z * 16.0f)));
            const auto fd = [](int v) { return v >= 0 ? v / 32 : -((-v + 31) / 32); };
            const glm::ivec2 want((fd(eb.x) - 2) * 32, (fd(eb.y) - 2) * 32);
            if (!fineJob.valid() && want != fineAt) {
                fineAt = want;
                fineJob = std::async(std::launch::async, [&tp, want, NONE] {
                    auto f = std::make_unique<FineData>();
                    f->origin = want;
                    constexpr int R = FINE + 4;   // кайма 2 блока: vb каймы смотрит на соседей
                    std::vector<int32_t> h(R * R), w(R * R);
                    world::ColumnHeights hm;
                    const auto fd2 = [](int v) { return v >= 0 ? v / 32 : -((-v + 31) / 32); };
                    for (int cz = fd2(want.y - 2); cz <= fd2(want.y + FINE + 1); ++cz)
                        for (int cx = fd2(want.x - 2); cx <= fd2(want.x + FINE + 1); ++cx) {
                            world::columnHeights(tp, cx, cz, hm);
                            for (int z = 0; z < 32; ++z)
                                for (int x = 0; x < 32; ++x) {
                                    const int i = cx * 32 + x - (want.x - 2), j = cz * 32 + z - (want.y - 2);
                                    if (i < 0 || j < 0 || i >= R || j >= R) continue;
                                    h[size_t(j) * R + i] = hm.h[z + 1][x + 1];
                                    w[size_t(j) * R + i] = hm.water[z + 1][x + 1];
                                }
                        }
                    constexpr int T = FINE + 2;
                    f->vb.assign(size_t(T) * T, NONE);
                    f->ground.assign(size_t(T) * T, 0);
                    f->wet.assign(size_t(T) * T, 0);
                    const auto surf = [&](int i, int j) { const size_t k = size_t(j) * R + i; return w[k] > h[k] ? float(w[k] + 1) : NONE; };
                    for (int j = 1; j < R - 1; ++j)
                        for (int i = 1; i < R - 1; ++i) {
                            const size_t o = size_t(j - 1) * T + (i - 1);
                            const float gr = float(h[size_t(j) * R + i] + 1), sv = surf(i, j);
                            f->ground[o] = gr;
                            f->wet[o] = sv > NONE;
                            if (sv > NONE) { f->vb[o] = sv; continue; }
                            float low = 1e9f;
                            for (int dj = -1; dj <= 1; ++dj)
                                for (int di = -1; di <= 1; ++di) { const float n = surf(i + di, j + dj); if (n > NONE) low = std::min(low, n); }
                            if (low < 1e9f) f->vb[o] = std::min(low, gr);
                        }
                    // неровность глади в 3×3 у блоков зоны
                    for (int j = 0; j < FINE; ++j)
                        for (int i = 0; i < FINE; ++i) {
                            float lo = 1e9f, hi = -1e9f;
                            for (int dj = 0; dj <= 2; ++dj)
                                for (int di = 0; di <= 2; ++di) {
                                    const float v = f->vb[size_t(j + dj) * T + i + di];
                                    if (v > NONE) { lo = std::min(lo, v); hi = std::max(hi, v); }
                                }
                            f->flat[j * FINE + i] = hi >= lo ? hi - lo : 0.0f;
                        }
                    return f;
                });
            }
            if (fineJob.valid() && fineJob.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                fineData = fineJob.get();
                const FineData& f = *fineData;
                constexpr int T = FINE + 2;
                std::vector<float> tex(size_t(T) * T * 2);
                for (size_t k = 0; k < size_t(T) * T; ++k) { tex[k * 2] = f.vb[k]; tex[k * 2 + 1] = f.ground[k]; }
                std::vector<uint8_t> mask(FINE * FINE, 0);
                std::vector<int32_t> blocks;
                const std::vector<float>& vel = shownVel.empty() ? fieldVel : shownVel;
                for (int j = 0; j < FINE; ++j)
                    for (int i = 0; i < FINE; ++i) {
                        if (!f.wet[size_t(j + 1) * T + i + 1]) continue;
                        const int bx = f.origin.x + i, bz = f.origin.y + j;
                        float sp = 0;
                        const int fx = (bx >> 4) - flowAt.x, fz = (bz >> 4) - flowAt.y;
                        if (!vel.empty() && fx >= 0 && fz >= 0 && fx < FLOW_SIZE && fz < FLOW_SIZE)
                            sp = std::hypot(vel[(size_t(fz) * FLOW_SIZE + fx) * 2], vel[(size_t(fz) * FLOW_SIZE + fx) * 2 + 1]);
                        if (f.flat[j * FINE + i] < 1.0f / 32.0f && sp < 1.5f) continue;
                        mask[size_t(j) * FINE + i] = 1;
                        blocks.push_back(bx);
                        blocks.push_back(bz);
                    }
                gpu->setFineWater(f.origin, FINE, tex, mask, blocks);
                simArea = glm::vec4(float(f.origin.x) / 16.0f, float(f.origin.y) / 16.0f, float(FINE) / 16.0f, 1.0f);
                journal::info("вода", "мелкая вода: зона ({}, {}), блоков {}", f.origin.x, f.origin.y, blocks.size() / 2);
            }
        }
        frame.reset();
        frame.clear(sky);
        frame.camera({viewProj, glm::vec3(0.4f, 1.0f, 0.25f), eye, voxels.window(),
                      float(lod.drawDistance) * 0.85f, float(lod.drawDistance), 9000.0f, 0.6f, sky,
                      std::chrono::duration<float>(now - startAt).count(), flowArea, simArea});
        chunksDrawn = voxels.draw(fr, frame);
        for (const OakMesh& m : oakMeshes) frame.drawFineChunk(m.mesh, m.base, m.fineOff);
        for (const auto& [mesh, origin] : rulerMeshes) frame.drawChunk(mesh, origin);
        const auto tFar = clock_t_::now();
        farTerrain.frame(lod, eye, fr, voxels, frame, farStats);
        part[1] = std::max(part[1], msSince(tFar));
        const auto tExe = clock_t_::now();
        gpu->execute(frame, fs);
        part[2] = std::max(part[2], msSince(tExe));
        // снимок: ждём, пока зона вокселей достроится (не дольше 2 минут), затем 5 кадров - снимок; с --fly -
        // 10 секунд облёта с замером отставания и времени кадра, снимок в конце; без --fly - замер 60 кадров
        if (shot) {
            if (!warm) {
                if (voxels.stats().pending == 0 || msSince(shotStart) > 120000) {
                    warm = true;
                    journal::info("мир", "зона вокселей готова за {:.0f} мс: столбцов {}, кусков {}, прямоугольников {}; работа потоков {:.0f} мс ({:.2f} мс на столбец)",
                                  msSince(shotStart), voxels.stats().ready, voxels.stats().chunks, voxels.stats().quads,
                                  voxels.stats().workMs, voxels.stats().workMs / std::max(1, voxels.stats().built));
                    shotStart = clock_t_::now();
                    part[0] = part[1] = part[2] = 0;
                    voxels.resetWorst();
                }
            } else if (recordSec > 0) {
                // запись: кадр каждые 1/15 с по часам (анимация идёт своим временем), затем выход
                static int recNo = 0;
                if (msSince(shotStart) >= recNo * 1000.0 / 15.0) {
                    char name[32];
                    std::snprintf(name, sizeof(name), "rec_%03d.bmp", recNo++);
                    saveShot(*gpu, width, height, name);
                }
                if (msSince(shotStart) > recordSec * 1000.0) running = false;
            } else {
                ++frameNo;
                const bool flying = flySpeed > 0;
                if (frameNo > 5 || flying) {
                    shotMs += double(dt) * 1000.0;
                    shotMaxMs = std::max(shotMaxMs, double(dt) * 1000.0);
                    maxPending = std::max(maxPending, voxels.stats().pending);
                }
                const bool done = flying ? msSince(shotStart) > 10000 : frameNo == 66;
                if ((!flying && frameNo == 5) || (flying && done)) {
                    saveShot(*gpu, width, height, "shot.bmp");
                    journal::info("окно", "кадр: кусков {}, прямоугольников {}; плиток {}, треугольников {}; вызовов {}",
                                  chunksDrawn, fs.quads, fs.tiles, fs.tileTriangles, fs.drawCalls);
                }
                if (done) {
                    const int n = flying ? frameNo : 60;
                    const int side = 2 * std::max(1, voxelBlocks / blocksPerColumn);
                    journal::info("окно", "время кадра ({} кадров, без ожидания экрана): среднее {:.2f} мс, худшее {:.2f} мс; не успевших столбцов - до {} из {}",
                                  n, shotMs / n, shotMaxMs, maxPending, side * side);
                    journal::info("окно", "худшее за кадр по частям: зона вокселей {:.2f} мс (сдвиг окна {:.2f}, очередь {:.2f}, выгрузка {:.2f}, маска {:.2f}), дальние плитки {:.2f} мс, отрисовка {:.2f} мс",
                                  part[0], voxels.stats().worstMs[0], voxels.stats().worstMs[1], voxels.stats().worstMs[2],
                                  voxels.stats().worstMs[3], part[1], part[2]);
                    running = false;
                }
            }
        }
        gpu->present();

        if (now - titleAt > std::chrono::milliseconds(500)) {
            const double ms = sumMs / frames;
            SDL_SetWindowTitle(window, std::format("Отмель — кадр {:.2f} мс | воксели: столбцов {} (ждут {}), кусков {} ({} прямоуг.) | "
                                                   "плиток {} ({} тр.) | вызовов {}",
                                                   ms, voxels.stats().ready, voxels.stats().pending, chunksDrawn, fs.quads,
                                                   fs.tiles, fs.tileTriangles, fs.drawCalls).c_str());
            farStats.built = 0;
            farStats.workMs = 0;
            sumMs = 0;
            frames = 0;
            titleAt = now;
        }
    }

    gpu.reset();
    SDL_DestroyWindow(window);
    SDL_Quit();
    journal::info("окно", "закрыто");
    return 0;
}
