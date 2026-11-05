// Отмель: точка входа. Окно, мир из зерна (карта высот, биомы - Р19): вблизи воксели, вдали гладкие плитки
// CDLOD (Р20); летающая камера.
// В заголовке окна - замеры (Р4): время кадра, генерации и сборки сеток, число кусков и прямоугольников.
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <format>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <unordered_set>
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
#include "gen/noise.h"
#include "world/caves.h"
#include "world/flow.h"
#include "world/forest.h"
#include "world/lod.h"
#include "world/mesher.h"
#include "world/oak.h"
#include "world/query.h"
#include "world/spawn.h"
#include "world/water_sim.h"
#include "world/terrain.h"
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "builder.h"
#include "player.h"
#include "game/items.h"
#include "game/felling.h"
#include "game/tree_body.h"
#include "game/save.h"
#include "game/stoneage.h"
#include <optional>
#include "dev_text.h"
#include "devinfo.h"
#include "voxel_zone.h"

namespace {

using clock_t_ = std::chrono::steady_clock;

// Р62: дуб в сетки мелких кусков (32³ мелких вокселей = 2×2×2 блока) от блока-основы; корни, чей мелкий воксель
// внутри блока земли (ground - верх земли блока-столбца), не рисуются. Только процессор - можно в фоне.
struct OakChunk { std::vector<uint32_t> verts; glm::ivec3 fineOff; int scale = 1; };
// Р91: дерево (world/tree) в сетки кусков 32³ вокселей размера scale от блока-основы; fineShift - сдвиг ствола в
// мелких вокселях внутри блока-основы. Листва у крупных деревьев - не мельче treeLeafScale: у великана она иначе
// уходит в десятки миллионов вокселей. Корни в толще земли не рисуются (ground - верх земли блока-столбца).
// Р93: keep(x, y, z) - оставить ли воксель (середина, мелкие воксели от основания ствола): зарубка, пень, падающая часть
struct KeepAll { bool operator()(float, float, float, int) const { return true; } };
template <class Ground, class Keep = KeepAll>
int buildTreeChunks(const world::TreeSkel& sk, glm::ivec3 base, glm::ivec3 fineShift, Ground ground, std::vector<OakChunk>& out,
                    int scale, bool leaves, Keep keep = {}, const game::TreeCut* leafCut = nullptr) {
    static thread_local world::ColumnHeights air;
    static thread_local bool airReady = false;
    if (!airReady) {
        for (auto& row : air.h) for (int32_t& v : row) v = INT32_MIN / 4;
        for (auto& row : air.water) for (int32_t& v : row) v = world::NO_WATER;
        airReady = true;
    }
    const auto fdiv = [](int32_t v, int32_t d) { return v >= 0 ? v / d : -((-v + d - 1) / d); };
    constexpr int SINK = 3;   // комель утоплен на 3 мелких вокселя
    static thread_local std::vector<world::Material> vox(32 * 32 * 32);
    static thread_local std::vector<uint16_t> br(32 * 32 * 32);
    int quads = 0;
    auto c = std::make_unique<world::Chunk>();
    const int leafScale = std::max(scale, world::treeLeafScale(sk.age));
    // проход 0 - древесина своим размером (и листва, если её размер тот же); проход 1 - листва крупнее
    for (int pass = 0; pass < 2; ++pass) {
        if (pass == 1 && !(leaves && leafScale != scale)) break;
        const bool wood = pass == 0;
        const bool leafHere = leaves && (leafScale == scale ? pass == 0 : pass == 1);
        const int s = pass == 0 ? scale : leafScale;
        int lo[3], hi[3];
        world::treeChunkRange(sk, s, lo, hi);
        const world::TreeIndex ix = world::treeIndex(sk, s);
        for (int cz = lo[2]; cz <= hi[2]; ++cz)
            for (int cx = lo[0]; cx <= hi[0]; ++cx)
                for (int cy = lo[1]; cy <= hi[1]; ++cy) {
                    if (!world::treeChunk(sk, ix, cx, cy, cz, wood, leafHere, vox.data(), br.data())) continue;
                    c->pos = {cx, cy, cz};
                    c->edited = true;
                    const glm::ivec3 off(cx * 32 * s + fineShift.x, cy * 32 * s - SINK, cz * 32 * s + fineShift.z);
                    bool any = false;
                    for (int y = 0; y < 32; ++y)
                        for (int z = 0; z < 32; ++z)
                            for (int x = 0; x < 32; ++x) {
                                const size_t k = (size_t(y) * 32 + z) * 32 + x;
                                world::Material m = vox[k];
                                if (m != world::AIR && br[k] != 0xFFFF && sk.branches[br[k]].root) {
                                    const int fx = off.x + x * s + s / 2, fy = off.y + y * s + s / 2, fz = off.z + z * s + s / 2;
                                    if (base.y + fdiv(fy, 16) <= ground(base.x + fdiv(fx, 16), base.z + fdiv(fz, 16))) m = world::AIR;
                                }
                                // Р102: листва осыпалась от ударов - у части ветвей её больше нет
                                if ((m == world::LEAVES || m == world::LEAVES2) && leafCut && br[k] != 0xFFFF && !leafCut->leafStays(int(br[k])))
                                    m = world::AIR;
                                if (m != world::AIR) {
                                    const int kv = int(keep(float((cx * 32 + x) * s) + s * 0.5f, float((cy * 32 + y) * s) + s * 0.5f,
                                                            float((cz * 32 + z) * s) + s * 0.5f, br[k] == 0xFFFF ? -1 : int(br[k])));
                                    if (!kv) m = world::AIR;
                                    else if (kv == 2 && (m == world::BARK || m == world::BARK2)) m = world::WOOD;   // Р96: окорено
                                }
                                c->at(x, y, z) = m;
                                any |= m != world::AIR;
                            }
                    if (!any) continue;
                    OakChunk oc;
                    world::MeshStats ms;
                    world::buildMesh(*c, air, oc.verts, ms);
                    if (oc.verts.empty()) continue;
                    quads += ms.quads;
                    oc.fineOff = off;
                    oc.scale = s;
                    out.push_back(std::move(oc));
                }
    }
    return quads;
}
double msSince(clock_t_::time_point t) { return std::chrono::duration<double, std::milli>(clock_t_::now() - t).count(); }

// Р73: фоновые задания кадра (поле течений, мелкая вода, зона леса, спрайты, сетки дубов) - свои постоянные потоки
// с очередью. std::async у MSVC отдаёт задание пулу потоков Windows, и сам запуск занимал главный поток до 150 мс
// (рывки в полёте над лесом, в OpenGL и Vulkan одинаково); здесь запуск - положить в очередь.
class Jobs {
public:
    explicit Jobs(unsigned n) { for (unsigned i = 0; i < n; ++i) threads_.emplace_back([this] { run(); }); }
    ~Jobs() {
        {
            std::lock_guard l(mu_);
            stop_ = true;
            queue_.clear();   // не начатые - не нужны: программа закрывается
        }
        cv_.notify_all();
        for (auto& t : threads_) t.join();
    }
    template <class F>
    auto submit(F&& f) -> std::future<std::invoke_result_t<F>> {
        using R = std::invoke_result_t<F>;
        auto task = std::make_shared<std::packaged_task<R()>>(std::forward<F>(f));
        auto fut = task->get_future();
        {
            std::lock_guard l(mu_);
            queue_.emplace_back([task] { (*task)(); });
        }
        cv_.notify_one();
        return fut;
    }

private:
    void run() {
        // Приоритет этих групп НЕ понижаем: кадр их дожидается (зоны, спрайты леса), и с пониженным
        // приоритетом он ждёт дольше. Пробовали понижать - замер не показал улучшения, откатили.
        for (;;) {
            std::function<void()> job;
            {
                std::unique_lock l(mu_);
                cv_.wait(l, [&] { return stop_ || !queue_.empty(); });
                if (stop_) return;
                job = std::move(queue_.front());
                queue_.pop_front();
            }
            job();
        }
    }
    std::mutex mu_;
    std::condition_variable cv_;
    std::deque<std::function<void()>> queue_;
    std::vector<std::thread> threads_;
    bool stop_ = false;
};

// Р73: забрать готовое у фоновой задачи; если get() ждал дольше 5 мс - в журнал
template <class T>
T take(std::future<T>& f, const char* what) {
    const auto t0 = clock_t_::now();
    T v = f.get();
    if (const double ms = msSince(t0); ms > 5.0) journal::warn("кадр", "{}: get() ждал {:.0f} мс", what, ms);
    return v;
}

// Дальний рельеф (Р20): плитки CDLOD. Каждый кадр - выбор по расстоянию, отсечение по пирамиде видимости.
// Недостающие плитки заказываются общему пулу (Р21); пока плитки нет, её место закрывает та же четверть
// плитки-родителя - грубее, но без дыры. Лишние плитки вытесняются.
class FarTerrain {
public:
    struct Stats { int built = 0; double workMs = 0; size_t cached = 0; int substituted = 0; };

    FarTerrain(render::Backend& gpu, Builder& builder) : gpu_(gpu), builder_(builder) {}
    ~FarTerrain() { release(); }
    // сетки - из видеопамяти до того, как закроется путь отрисовки
    void release() { for (auto& [k, e] : cache_) gpu_.destroyMesh(e.mesh); cache_.clear(); }

    void frame(const world::LodSettings& s, glm::vec3 eye, const Frustum& fr, const VoxelZone& voxels,
               render::CommandList& out, Stats& st) {
        ++frame_;
        // готовые плитки - в видеопамять (дёшево: 13 КБ на плитку)
        Builder::TileResult r;
        for (int n = 0; n < 512 && builder_.popTile(r); ++n) {
            Entry e;
            e.mesh = gpu_.uploadTile(r.mesh.data, r.mesh.water);
            e.falls = std::move(r.mesh.falls);   // Р75: места падения воды - брызги вдали
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
    struct Entry { render::MeshId mesh = render::NO_MESH; float minY = 0, maxY = 0; uint64_t used = 0; std::vector<float> falls; };

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

public:
    // Р75: места падения воды у плиток, что рисуются (выбор прошлого кадра): по 6 чисел, см. world::TileMesh::falls
    template <class F>
    void forEachFall(F&& f) const {
        for (const world::TileDraw& d : sel_) {
            const auto it = cache_.find(world::tileHash(d.key));
            if (it == cache_.end()) continue;
            const std::vector<float>& v = it->second.falls;
            for (size_t k = 0; k + 6 <= v.size(); k += 6) f(&v[k]);
        }
    }
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
    journal::installCrashHandler();   // Р64
    // Ключи (пока вместо меню настроек):
    //   --voxels B   зона вокселей: квадрат ±B блоков вокруг точки старта (по умолчанию 128) - дальше гладко (Р20)
    //   --draw B     дальность видимости, блоков (24576)
    //   --detail K   детальность дальних уровней: дальность уровня 0, блоков (128; не меньше 65 - иначе щели)
    //   --at X Z     точка старта, блоки;  --eye x y z yaw pitch - камера;  --shot - снимок и выход
    //   --fly V      (со --shot) облёт: камера летит вперёд со скоростью V блоков/с 10 секунд, замер отставания
    //   --gl         путь отрисовки OpenGL (запасной; по умолчанию - Vulkan, Р73)
    bool shot = false, vulkan = true;
    float recordSec = 0;   // со --shot: запись кадров rec_NNN.bmp 15 раз в секунду столько секунд
    int voxelBlocks = 128;
    float atX = 0, atZ = 0, flySpeed = 0, spinRate = 0;
    bool winter = false, hold = false, autoChop = false, newWorld = false, forceSave = false, giveTools = false;
    float startPitch = -0.35f;
    world::LodSettings lod;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--shot") shot = true;
        if (a == "--gl") vulkan = false;
        if (a == "--voxels" && i + 1 < argc) voxelBlocks = std::max(2, std::stoi(argv[i + 1]));
        if (a == "--draw" && i + 1 < argc) lod.drawDistance = std::max(256, std::stoi(argv[i + 1]));
        if (a == "--detail" && i + 1 < argc) lod.detail = std::max(65, std::stoi(argv[i + 1]));
        if (a == "--at" && i + 2 < argc) { atX = std::stof(argv[i + 1]); atZ = std::stof(argv[i + 2]); }
        if (a == "--fly" && i + 1 < argc) flySpeed = std::stof(argv[i + 1]);
        if (a == "--record" && i + 1 < argc) recordSec = std::stof(argv[i + 1]);
        if (a == "--winter") winter = true;
        if (a == "--hold") hold = true;
        if (a == "--tools") giveTools = true;   // Р99: инструменты у старта (для проверки; в игре их надо сделать)
        if (a == "--new") newWorld = true;
        if (a == "--save") forceSave = true;   // Р98: сохранять и в режиме снимка (проверка)   // Р98: не загружать сохранение (оно перезапишется)
        if (a == "--chop") autoChop = true;   // Р93: со --shot - срубить ближайшее дерево (проверка)
        if (a == "--pitch" && i + 1 < argc) startPitch = std::stof(argv[i + 1]);   // наклон камеры на старте   // Р92: топор в правой руке с начала (для снимка)   // Р91: деревья без листвы
        if (a == "--spin" && i + 1 < argc) spinRate = std::stof(argv[i + 1]);   // со --shot: поворот камеры, рад/с
    }
    if (!SDL_Init(SDL_INIT_VIDEO)) { journal::error("окно", "SDL не запустился: {}", SDL_GetError()); return 1; }
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 6);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG);
    // глубина и сглаживание - в своём буфере кадра (Р20), окну нужен только цвет
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 0);

    int width = 1280, height = 720;
    const std::string shaderDir = std::string(SDL_GetBasePath()) + "shaders";
    SDL_Window* window = nullptr;
    std::unique_ptr<render::Backend> gpu;
    if (vulkan) {   // Р73: Vulkan; не поднялся (нет драйвера 1.3) - окно заново, под OpenGL
        window = SDL_CreateWindow("Отмель", width, height, SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE);
        if (window) gpu = render::createVulkan(window, shaderDir.c_str());
        if (!gpu) {
            journal::warn("окно", "Vulkan не поднялся ({}) - запасной путь OpenGL", window ? "см. выше" : SDL_GetError());
            if (window) SDL_DestroyWindow(window);
            window = nullptr;
        }
    }
    if (!gpu) {
        window = SDL_CreateWindow("Отмель", width, height, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
        if (!window) { journal::error("окно", "окно не создано: {}", SDL_GetError()); return 1; }
        gpu = render::createGL(window, shaderDir.c_str());
    }
    if (!gpu) return 1;

    // Зона вокселей едет за камерой (Р21): ±R столбцов кусков (столбец - 2×2 блока); строят фоновые потоки
    const int blocksPerColumn = world::CHUNK / world::VOXELS_PER_BLOCK;
    world::TerrainParams tp;
    for (int i = 1; i + 1 < argc; ++i)
        if (std::string(argv[i]) == "--style") tp.mountainStyle = std::stoi(argv[i + 1]);   // вид гор (Р23)
    // Р83: правки игрока - сломанные блоки. Их видит и чтение по вокселю, и постройка сеток.
    world::Edits edits;
    Builder builder(tp);
    builder.setEdits(&edits);
    VoxelZone voxels(*gpu, builder, std::max(1, voxelBlocks / blocksPerColumn));
    voxels.setEdits(&edits);   // Р83: по числу правок зона отличает свежую сетку от устаревшей
    FarTerrain farTerrain(*gpu, builder);

    // Камера: в блоках. Мышь - взгляд, WASD - движение, пробел/Ctrl - вверх/вниз, Shift - быстрее,
    // колесо - скорость, L - раскраска дальних уровней, Esc - выход. Старт - в 48 блоках мира над землёй (Р54: 3 прежних).
    // Р81: точка старта - на суше и на настоящей поверхности (world/spawn.h), а не на высоте
    // исходного рельефа: над ним бывает вода, а у выхода шахты земля срезана (Р79).
    const world::Spawn spawn = world::findSpawn(tp, atX, atZ);
    if (spawn.moved)
        journal::info("мир", "в точке {:.0f}, {:.0f} вода - старт перенесён на сушу: {:.0f}, {:.0f}", atX, atZ, spawn.x, spawn.z);
    else if (!spawn.dry)
        journal::warn("мир", "вокруг {:.0f}, {:.0f} суши нет - старт над водой", atX, atZ);
    atX = float(spawn.x);
    atZ = float(spawn.z);
    const float ground = float(spawn.ground);
    // ПОЛОЖЕНИЕ КАМЕРЫ - С ДВОЙНОЙ ТОЧНОСТЬЮ (Р79).
    //
    // Числом одинарной точности на удалении в сто тысяч блоков шаг представимости около 0.008 блока,
    // а на двухстах тысячах - 0.016. Шаг камеры за кадр - примерно 0.003 блока, то есть МЕНЬШЕ этого
    // шага: прибавка по одной оси копилась и срывалась скачком, по другой - вдвое реже и вдвое крупнее.
    // Отсюда были рывки и ощущение, что по одной оси тянет охотнее, чем по другой. Копим в double,
    // а в отрисовку отдаём уже разность с началом координат - она мала, и точности хватает с запасом.
    glm::dvec3 eye(atX, ground + 2.0, atZ);   // Р81: глаза чуть выше поверхности, а не в её толще
    // Р83: игрок с телом. Пешком - притяжение, прыжок, бег; двойной прыжок - в полёт и обратно.
    // Правки мира (сломанные блоки) видит и чтение по вокселю, и постройка сеток.
    world::VoxelQuery voxelQuery(tp, &edits);
    game::Player player;
    player.flying = shot;   // снимок не должен падать на землю; живая игра начинается пешком
    bool jumpWas = false;
    const auto eyeF = [&eye] { return glm::vec3(eye); };
    float yaw = -0.6f, pitch = startPitch, speedScale = 1.0f;
    // Р92: предметы в мире и руки (0 - левая, 1 - правая). На старте рядом лежат инструменты.
    game::Items items;
    std::optional<game::Item> hands[2];
    if (hold) hands[1] = game::makeTool(game::ItemKind::AXE);
    struct HandMesh { const game::ItemModel* model = nullptr; std::vector<render::MeshId> meshes; std::vector<glm::ivec3> offs; };
    HandMesh handMesh[2];
    bool toolsSpawned = false;
    // Р99: каменный век. Тёска - камень у глаз (E); крафт - сетка 2×2 ячеек по четверть блока на земле (F);
    // природные камни и сушняк вокруг игрока (подобранное помнится - не возвращается).
    std::optional<game::Knap> knap;
    // Р100: крафт на земле - F2: рамка едет за прицелом; F2 - встала; F2 - убрать. stage: 0 нет, 1 едет, 2 стоит
    struct Craft { int stage = 0; glm::dvec3 corner{0}; };
    Craft craft;
    std::unordered_set<uint64_t> takenNatural;
    std::unordered_map<uint64_t, uint32_t> naturalSpawned;   // ключ места -> id предмета
    std::unordered_map<uint64_t, int> fiberHits;            // соскоблено с дерева (по месту)
    float naturalAt = 1e9f;
    float handTime = 0, walkPhase = 0, walkAmp = 0;   // покачивание предмета в руке
    float handAge[2] = {0, 0};   // Р104: сколько предмет в руке - двойным щелчком только что взятое не бросаем
    // Р93: рубка. Зарубки деревьев (по месту в лесу - переживают перестройку зоны), падающие и лежащие стволы,
    // щепки, замах топора.
    std::unordered_map<uint64_t, game::TreeCut> treeCuts;
    struct FallenTree { game::TreeBody tb; int contour = -1; std::vector<render::MeshId> cMeshes; std::vector<glm::ivec3> cOffs; glm::vec3 cCorner{0};
                        const game::ItemModel* cModel = nullptr; std::shared_ptr<game::ItemModel> cHold;
                        std::vector<render::MeshId> meshes; std::vector<glm::ivec3> offs; std::vector<int> scales; };
    std::vector<FallenTree> fallen;
    // Р94: ОСЯЗАЕМОСТЬ. Блоки, занятые стволами: неподвижные (стоящие деревья, пни) и все вместе с поваленными.
    // Первые - опора для падающих тел, вторые - для игрока и предметов (VoxelQuery::extra).
    std::unordered_set<uint64_t> staticTrunks, allTrunks;
    struct Chip { glm::vec3 p, v; float life; int kind = 5; };   // Р102: 5 - щепка, 6 - лист
    std::vector<Chip> chips;
    float swing[2] = {0, 0};   // замах руки, секунды до конца
    constexpr float SWING = 0.5f;
    struct PendingChop { int hand; glm::dvec3 start, dir; double tBlock; float t; };
    std::optional<PendingChop> pendingChop;
    float kick = 0;            // отдача удара - вздрагивание камеры
    std::vector<char> crushed; // упавшее дерево уже смято по земле
    // Подсказки разработчика (devinfo.h): что под прицелом и переход по координатам
    dev::Aim aim;
    std::string gotoText;
    bool typing = false;
    bool showPanel = true;                 // F3 - показать или убрать панель отладки
    std::vector<std::string> panelLines;   // что на ней написано сейчас
    dev::TextImage panelImage;
    int debug = 0;
    for (int i = 1; i + 5 < argc; ++i)
        if (std::string(argv[i]) == "--eye") {
            eye = {std::stod(argv[i + 1]), std::stod(argv[i + 2]), std::stod(argv[i + 3])};
            yaw = std::stof(argv[i + 4]);
            pitch = std::stof(argv[i + 5]);
        }
    // Р53: дубы-коряги для просмотра: левая кнопка - новый дуб в 32 блоках перед камерой, правая - убрать все,
    // R - корни под землёй прятать / показывать. Сетка - кусками 32³ поверх мира (в мир пока не вписаны).
    struct OakSpot { int32_t x, y, z; uint32_t seed; };
    struct OakMesh { render::MeshId mesh; glm::ivec3 base, fineOff; int scale = 1; };
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
    // Р60: роща - n дубов в круге радиусом r блоков у точки (cx, cz), не ближе 5 блоков друг к другу (бросками),
    // у каждого своё семя; основа - на земле под комлем
    const auto plantGrove = [&](int32_t cx, int32_t cz, int n, int r, uint32_t seed) {
        std::vector<std::pair<int32_t, int32_t>> at;
        uint32_t h = seed * 2654435761u + 12345u;
        const auto next = [&h] { h ^= h << 13; h ^= h >> 17; h ^= h << 5; return h; };
        for (int tries = 0; tries < n * 40 && int(at.size()) < n; ++tries) {
            const float a = float(next() % 6283) / 1000.0f, d = r * std::sqrt(float(next() % 10000) / 10000.0f);
            const int32_t x = cx + int32_t(std::cos(a) * d), z = cz + int32_t(std::sin(a) * d);
            bool ok = true;
            for (const auto& [px, pz] : at) ok &= (px - x) * (px - x) + (pz - z) * (pz - z) >= 25;
            if (!ok) continue;
            at.push_back({x, z});
            oaks.push_back({x, oakBase(x, z), z, next()});
        }
    };
    for (OakSpot& o : oaks) o.y = oakBase(o.x, o.z);
    for (int i = 1; i + 3 < argc; ++i)
        if (std::string(argv[i]) == "--grove")
            plantGrove(int32_t(std::floor(std::stof(argv[i + 1]) * 16)), int32_t(std::floor(std::stof(argv[i + 2]) * 16)),
                       std::stoi(argv[i + 3]), 40, 7);
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
    if (shot) gpu->setVsync(false);
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
    int64_t farBuiltTotal = 0;
    auto last = clock_t_::now(), titleAt = last;
    const auto startAt = last;
    // Р43: поле течений вокруг камеры - в фоне; пересчёт, когда камера ушла от центра поля на четверть
    constexpr int FLOW_SIZE = 256;
    struct FlowData { std::vector<float> vel, ground, surface; };
    // Р73: задания зон (поле, мелкая вода, зона леса, спрайты) - своя очередь: не стоят за сотнями сеток дубов
    Jobs zoneJobs(2), treeJobs(std::max(2u, std::thread::hardware_concurrency() / 2));
    Jobs effectJobs(1);   // Р75: кромки воды - своя очередь: первый кадр ждёт поле, лес и спрайты, но не их
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
    struct Drop { glm::vec3 p, v; float life; int kind; float size; float y0 = 0; };   // kind: 0 капля, 1 пыль, 2 падающая масса, 3 клочья пены (Р75); size - тексели; y0 - откуда упала
    std::vector<Drop> drops;
    bool sprayWarmed = false;   // Р75: брызги прогреты по кромкам
    float dropRate = 1.0f, dropDemand = 0.0f;   // Р75: общий множитель частоты; ожидаемое число капель при полной частоте
    std::vector<float> flowGround, flowSurface, dropVerts;
    // Р58: мелкая вода у камеры - зона 128×128 блоков (4×4 куска); уровни по блокам - в фоне при смене куска камеры
    constexpr int FINE = 128;
    constexpr float NONE = -1e9f;
    struct FineData { glm::ivec2 origin; std::vector<float> vb, ground; std::vector<uint8_t> wet; float flat[FINE * FINE]; std::vector<int32_t> blocks, sources; };
    std::future<std::unique_ptr<FineData>> fineJob;
    glm::ivec2 fineAt(INT32_MIN / 2);
    std::unique_ptr<FineData> fineData;
    // Р75: кромки падения воды поблочно в 1024×1024 блоков у камеры (фон, сдвиг - при уходе на 256 блоков): брызги
    // и падающая вода - по точным кромкам; там же верх глади (или земли) - куда падают блоки воды
    constexpr int LIPS = 1024, LIPG = LIPS + 64;   // сетка - с каймой в кусок (соседи у края); ±512 блоков
    struct LipData { glm::ivec2 origin; std::vector<float> top; std::vector<uint8_t> wet; std::vector<float> lips; };   // lips: x, z, верх, перепад, dx, dz
    std::future<std::unique_ptr<LipData>> lipJob;
    std::unique_ptr<LipData> lipData;
    glm::ivec2 lipAt(INT32_MIN / 2);
    // Р62: лес в генерации - зона 7×7 кусков (224 блока) вокруг куска камеры; места - в фоне, деревья - в фоне по 6
    struct ForestSpot { uint64_t key; glm::ivec3 base; glm::ivec3 shift; uint32_t seed; world::TreeAge age; };
    struct ForestZone { glm::ivec2 origin; int side; std::shared_ptr<std::vector<int32_t>> ground; std::vector<float> skyTop; std::vector<ForestSpot> spots; int nWet = 0, nSteep = 0, nSnow = 0, nSparse = 0, nCave = 0; float rainSum = 0, groveSum = 0; int nC = 0; };
    struct ForestBuilt { uint64_t key; int level; glm::ivec3 base; std::vector<OakChunk> chunks; std::shared_ptr<const world::TreeSkel> skel; bool leaves; };
    // Р63: дерево леса - текущий уровень детализации (-1 - нет), какой строится, его сетки
    // Р91: skel - выращенный скелет (держим: по нему же потом рубка); leaves - с листвой ли нынешняя сетка
    struct ForestTree { ForestSpot spot; int level = -1, building = -1; std::vector<OakMesh> meshes; std::shared_ptr<const world::TreeSkel> skel; bool leaves = true; bool dirty = false; };
    bool leavesOn = !winter;   // Р91: листва (времена года - потом; пока L переключает для проверки)
    std::future<std::unique_ptr<ForestZone>> forestJob;
    glm::ivec2 forestAt(INT32_MIN / 2);
    std::shared_ptr<std::vector<int32_t>> forestGround;
    glm::ivec2 forestGroundOrigin(0);
    int forestGroundSide = 0;
    glm::vec4 skyArea(0);   // Р85: угол и сторона карты верхних высот (блоки)
    std::unordered_map<uint64_t, ForestTree> forest;
    std::vector<std::future<ForestBuilt>> forestRunning;
    glm::vec3 forestEyePrev(0);   // Р70: скорость камеры
    auto spritesAt = clock_t_::now();
    // Р65: спрайты дубов вдали - в фоне, при сдвиге камеры на 64 блока или смене поля течений и зоны леса
    std::future<std::vector<float>> spriteJob;
    glm::vec2 spriteAt(1e9f);
    glm::ivec2 spriteZoneAt(INT32_MIN);
    auto spriteCache = std::make_shared<std::unordered_map<uint64_t, world::SpriteRegion>>();   // Р74: только задание спрайтов
    size_t spriteCount = 0;
    std::vector<float> spriteFar;      // спрайты кольца (фон)
    bool spritesDirty = false;         // дубы зоны без сетки - тоже спрайтами (пока строятся)
    bool firstFrame = true;            // Р66: первый кадр ждёт лес - видно сразу
    // Р64: замеры для журнала - долгие кадры и сводка раз в 10 с
    int forestUploads = 0, fineShown = 0;
    double statSum = 0, statWorst = 0;
    int statFrames = 0;
    auto statAt = clock_t_::now(), spikeAt = clock_t_::now();
    uint32_t dropRng = 0x9E3779B9u;
    const auto rnd = [&dropRng] { dropRng ^= dropRng << 13; dropRng ^= dropRng >> 17; dropRng ^= dropRng << 5; return float(dropRng >> 8) / 16777216.0f; };
    int simFrame = 0;
    glm::vec4 simArea(0.0f);
    double sumMs = 0;
    int frames = 0, chunksDrawn = 0;
    bool running = true;
    // Р73: разбивка кадра по частям главного цикла - в журнал долгого кадра (где ушло время)
    static const char* const SECT[] = {"события", "зона вокселей", "поле течений", "брызги", "мелкая вода", "лес",
                                       "спрайты", "список кадра", "отрисовка", "снимок", "показ"};
    constexpr int NSECT = int(std::size(SECT));
    double sect[NSECT] = {};
    auto lapAt = clock_t_::now();
    const auto lap = [&](int i) { const auto t = clock_t_::now(); sect[i] += std::chrono::duration<double, std::milli>(t - lapAt).count(); lapAt = t; };
    // Р98: СОХРАНЕНИЕ - загрузка при старте (кроме режима снимка и --new), запись раз в минуту, по F5 и на выходе
    const std::string savePath = std::string(SDL_GetBasePath()) + "saves/world.bin";
    const bool useSave = (!shot || forceSave) && !newWorld;
    if (useSave) {
        game::SaveData sd;
        if (game::loadWorld(savePath, sd) && sd.seed == tp.seed) {
            eye = sd.eye;
            yaw = sd.yaw;
            pitch = sd.pitch;
            player.flying = sd.flying;
            player.eye = eye;
            for (const world::Edit& e : sd.edits) edits.set(e.x, e.y, e.z, e.m);
            for (int h = 0; h < 2; ++h) hands[h] = sd.hands[h];
            for (auto& [key, c] : sd.cuts) treeCuts[key] = c;
            for (game::TreeBody& tb : sd.bodies) { FallenTree ft; ft.tb = std::move(tb); fallen.push_back(std::move(ft)); }
            for (const game::WorldItem& it : sd.items) {
                items.spawn(it.item, it.body.pos, it.body.rot, it.body.vel, it.body.ang, it.placed);
                items.list.back().body.asleep = it.body.asleep || it.placed;
            }
            toolsSpawned = true;
            takenNatural.insert(sd.taken.begin(), sd.taken.end());
            journal::info("сохранение", "загружено: правок {}, деревьев {}, стволов {}, предметов {}", sd.edits.size(), sd.cuts.size(),
                          sd.bodies.size(), sd.items.size());
        }
    }
    const auto saveNow = [&] {
        if (!useSave) return;
        game::SaveData sd;
        sd.seed = tp.seed;
        sd.eye = eye;
        sd.yaw = yaw;
        sd.pitch = pitch;
        sd.flying = player.flying;
        for (const auto& [k, e] : *edits.snapshot()) sd.edits.push_back(e);
        for (int h = 0; h < 2; ++h) sd.hands[h] = hands[h];
        for (const auto& [key, c] : treeCuts) sd.cuts.push_back({key, c});
        for (const FallenTree& ft : fallen) sd.bodies.push_back(ft.tb);
        for (const game::WorldItem& w : items.list) if (!w.nat) sd.items.push_back(w);   // Р99: природное вырастет само
        sd.taken.assign(takenNatural.begin(), takenNatural.end());
        const auto t0 = clock_t_::now();
        if (game::saveWorld(savePath, sd))
            journal::info("сохранение", "мир сохранён за {:.0f} мс: правок {}, деревьев {}, стволов {}, предметов {}", msSince(t0),
                          sd.edits.size(), sd.cuts.size(), sd.bodies.size(), sd.items.size());
        else journal::warn("сохранение", "не удалось записать {}", savePath);
    };
    float sinceSave = 0;
    // Р102: от удара по дереву с кроны сыплются листья (center, radius - блоки)
    const auto dropLeaves = [&](glm::dvec3 center, float radius, int n) {
        static uint32_t lr = 4242u;
        const auto rnd = [] { lr = lr * 1664525u + 1013904223u; return float(lr >> 8) / 16777216.0f; };
        for (int k = 0; k < n; ++k) {
            const glm::vec3 off((rnd() - 0.5f) * 2.0f * radius, (rnd() - 0.3f) * radius, (rnd() - 0.5f) * 2.0f * radius);
            chips.push_back({glm::vec3(center) + off, glm::vec3(rnd() - 0.5f, -0.3f, rnd() - 0.5f), 3.0f + 2.0f * rnd(), 6});
        }
    };
    // Р100: КРАФТ НА ЗЕМЛЕ. Ячейки 2×2 по четверть блока от угла corner; пятая (готовое) - справа от них.
    // Что в ячейке - решает центр масс предмета, лежащего в ней; порядок и поворот не важны.
    const auto craftCell = [&](glm::dvec3 st, glm::dvec3 dr) -> int {   // какая ячейка под прицелом (0..3, 4 - готовое, -1)
        if (craft.stage != 2 || std::fabs(dr.y) < 1e-4) return -1;
        const double t = (craft.corner.y - st.y) / dr.y;
        if (t <= 0 || t > 5) return -1;
        const glm::dvec3 p = st + dr * t;
        const double u = (p.x - craft.corner.x) / 0.25, v = (p.z - craft.corner.z) / 0.25;
        if (u >= 0 && u < 2 && v >= 0 && v < 2) return int(v) * 2 + int(u);
        if (u >= 2.2 && u < 3.2 && v >= 0.5 && v < 1.5) return 4;
        return -1;
    };
    const auto craftMatch = [&](game::Item& out, std::string& name, std::vector<size_t>& idx) {
        std::vector<const game::Item*> got;
        idx.clear();
        for (size_t k = 0; k < items.list.size(); ++k) {
            const glm::dvec3 p = items.list[k].body.pos - craft.corner;
            if (p.x >= 0 && p.x < 0.5 && p.z >= 0 && p.z < 0.5 && p.y > -0.2 && p.y < 0.6) { got.push_back(&items.list[k].item); idx.push_back(k); }
        }
        return !got.empty() && game::matchRecipe(got, out, name);
    };
    // Р93-Р96: УДАР ИНСТРУМЕНТОМ из руки hand по лучу (start, dir - блоки); tBlock - где луч упёрся в рельеф.
    //   топор: сук лежащего дерева - обрубка; его ствол - раскряжёвка (медленно); стоящее - зарубка; пень - корни;
    //   пила: ствол лежащего - раскряжёвка; стоящее - зарубка (быстрее топора);
    //   скобель: ствол лежащего или бревно - окорка метра;
    //   лопата: пень без корней - выкопать.
    const auto chopTree = [&](int hand, glm::dvec3 start, glm::dvec3 dir, double tBlock) {
        static uint32_t cr = 777u;
        const auto rnd = [] { cr = cr * 1664525u + 1013904223u; return float(cr >> 8) / 16777216.0f; };
        if (!hands[hand]) return;
        const game::ItemKind tool = hands[hand]->kind;
        const auto spray = [&](glm::vec3 wp, glm::vec3 away, int n) {
            for (int k = 0; k < n; ++k)
                chips.push_back({wp, away * (1.5f + 2.5f * rnd()) + glm::vec3(rnd() - 0.5f, 0.8f + rnd() * 1.5f, rnd() - 0.5f) * 3.0f, 1.0f + rnd()});
        };
        // поверхность над точкой (блоки): чтобы выпавшее из-под земли не застряло в ней
        const auto surfaceAbove = [&](glm::dvec3 p) {
            int32_t y = int32_t(std::floor(p.y));
            for (int k = 0; k < 48 && voxelQuery.solidTerrain(int32_t(std::floor(p.x)), y, int32_t(std::floor(p.z))); ++k) ++y;
            return std::max(p.y, double(y) + 0.6);
        };
        // ─── лежащие деревья
        {
            FallenTree* bf = nullptr;
            int bb = -1;
            float bt = 3.2f;
            glm::vec3 lp(0);
            for (FallenTree& ft : fallen) {
                if (!ft.tb.free) continue;
                float tt;
                glm::vec3 l;
                const int b = game::hitBranch(ft.tb, start, dir, bt, tt, &l);
                if (b >= 0 && tt < bt) { bt = tt; bb = b; bf = &ft; lp = l; }
            }
            if (bf && double(bt) < tBlock) {
                const glm::vec3 wp = glm::vec3(start + dir * double(bt));
                game::TreeBody& tb = bf->tb;
                kick = 0.03f;
                if (tb.cut.leafKeep > 0.15f) {   // Р102-Р103: лежащее осыпается от каждого удара - до 15%
                    tb.cut.leafKeep = std::max(0.15f, tb.cut.leafKeep * 0.75f);
                    tb.meshDirty = true;
                    dropLeaves(glm::dvec3(wp) + glm::dvec3(0, 1.0, 0), 2.0f, 20);
                }
                if (bb > 0 && tool == game::ItemKind::AXE) {   // обрубка сука
                    spray(wp, -glm::vec3(dir), 10);
                    if (tb.chopBranch != bb) { tb.chopBranch = bb; tb.chopHits = 0; }
                    ++tb.chopHits;
                    const int need = game::limbHits(tb, bb);
                    journal::info("рубка", "сук: удар {} из {}", tb.chopHits, need);
                    if (tb.chopHits >= need) {
                        game::Severed sv;
                        if (game::severBranch(tb, bb, sv)) {
                            items.spawn(sv.item, sv.pos, sv.rot, sv.vel + glm::vec3(0, 0.5f, 0), glm::vec3(0));
                            journal::info("рубка", "отрублено: {}", game::itemName(sv.item));
                        }
                        tb.chopBranch = -1;
                        bf->contour = -2;
                    }
                } else if (bb == 0 && (tool == game::ItemKind::AXE || tool == game::ItemKind::SAW)) {   // раскряжёвка
                    spray(wp, -glm::vec3(dir), tool == game::ItemKind::SAW ? 6 : 10);
                    const float ym = tb.yLo + std::round((lp.y - tb.yLo) / 16.0f) * 16.0f;
                    glm::vec2 c;
                    float R = 2;
                    if (!game::trunkAt(*tb.skel, ym, c, R)) return;
                    if (tb.chopMark != ym) { tb.chopMark = ym; tb.markHits = 0; }
                    ++tb.markHits;
                    const int need = tool == game::ItemKind::SAW ? std::max(2, int(2.0f * R / 3.0f + 0.5f)) : std::max(3, int(2.0f * R / 1.5f + 0.5f));
                    journal::info("рубка", "раскряжёвка: {} из {}", tb.markHits, need);
                    if (tb.markHits >= need) {
                        FallenTree up;
                        if (game::splitBody(tb, ym, up.tb)) {
                            bf->contour = -2;
                            journal::info("рубка", "ствол распилен на высоте {:.0f} м от реза", (ym - tb.cut.y0) / 16.0f);
                            fallen.push_back(std::move(up));   // tb и bf дальше недействительны
                            // голые куски ствола до 6 м - брёвна-предметы
                            for (size_t i = 0; i < fallen.size();) {
                                game::Severed sv;
                                if (game::isLog(fallen[i].tb) && game::bodyToLog(fallen[i].tb, sv)) {
                                    items.spawn(sv.item, sv.pos, sv.rot, sv.vel, glm::vec3(0));
                                    journal::info("рубка", "бревно: {}", game::itemName(sv.item));
                                    for (render::MeshId m : fallen[i].meshes) gpu->destroyMesh(m);
                                    for (render::MeshId m : fallen[i].cMeshes) gpu->destroyMesh(m);
                                    fallen.erase(fallen.begin() + long(i));
                                } else ++i;
                            }
                        }
                    }
                } else if (bb == 0 && tool == game::ItemKind::DRAWKNIFE) {   // окорка метра
                    const int strips = game::debarkBody(tb, lp.y);
                    for (int k = 0; k < strips; ++k)
                        items.spawn(game::makeBark(), glm::dvec3(wp) + glm::dvec3(rnd() - 0.5, 0.3, rnd() - 0.5),
                                    glm::angleAxis(rnd() * 6.28f, glm::vec3(0, 1, 0)), glm::vec3(rnd() - 0.5f, 1.5f, rnd() - 0.5f), glm::vec3(rnd(), rnd(), rnd()));
                    if (strips) journal::info("рубка", "окорено: стружек коры {}", strips);
                }
                return;
            }
        }
        // ─── стоящие деревья и пни
        ForestTree* best = nullptr;
        bool stump = false;
        float bestT = 3.2f * 16.0f;
        glm::vec3 hitP(0);
        for (auto& [key, t] : forest) {
            if (!t.skel) continue;
            const glm::dvec3 org = glm::dvec3(t.spot.base) * 16.0 + glm::dvec3(t.spot.shift.x, -3, t.spot.shift.z);
            const glm::dvec3 o = start * 16.0 - org;
            if (o.x * o.x + o.z * o.z > 80.0 * 80.0) continue;
            const auto ci = treeCuts.find(key);
            const bool felled = ci != treeCuts.end() && ci->second.felled;
            if (felled && ci->second.dug) continue;
            float th;
            glm::vec3 hp;
            if (game::hitTrunk(*t.skel, glm::vec3(o), glm::vec3(dir), bestT, th, hp, felled ? ci->second.y0 + 1.0f : 1e9f) && th < bestT) {
                bestT = th; best = &t; hitP = hp; stump = felled;
            }
        }
        if (!best || bestT / 16.0f >= tBlock) return;
        game::TreeCut& cut = treeCuts[best->spot.key];
        const glm::dvec3 org = glm::dvec3(best->spot.base) * 16.0 + glm::dvec3(best->spot.shift.x, -3, best->spot.shift.z);
        const glm::vec3 wp = glm::vec3((org + glm::dvec3(hitP)) / 16.0);
        const world::TreeSkel& sk = *best->skel;
        if (stump) {
            if (tool == game::ItemKind::AXE && !cut.allRootsCut()) {
                // корни: ближайший по направлению удара ещё целый корень от комля
                const glm::vec2 h = glm::vec2(hitP.x, hitP.z) - cut.center;
                int bestR = -1, bestBr = -1;
                float bd = -2;
                for (size_t b = 1; b < sk.branches.size(); ++b) {
                    const world::TreeBranch& br = sk.branches[b];
                    if (!br.root || br.first < 0 || b >= cut.rootOf.size()) continue;
                    const world::TreeNode& at = sk.nodes[size_t(sk.nodes[size_t(br.first)].parent)];
                    if (at.branch != 0) continue;
                    const int ri = cut.rootOf[b];
                    if (ri < 0 || (cut.rootsCut >> ri & 1u)) continue;
                    const world::TreeNode& f = sk.nodes[size_t(br.first)];
                    const glm::vec2 rd = glm::normalize(glm::vec2(f.p[0], f.p[2]) - cut.center + glm::vec2(1e-3f));
                    const float dd = glm::length(h) > 1e-3f ? glm::dot(glm::normalize(h), rd) : 0.0f;
                    if (dd > bd) { bd = dd; bestR = ri; bestBr = int(b); }
                }
                if (bestR < 0) return;
                spray(wp, -glm::vec3(dir), 8);
                kick = 0.03f;
                if (cut.rootTarget != bestR) { cut.rootTarget = bestR; cut.rootHits = 0; }
                ++cut.rootHits;
                const int need = std::max(1, int(2.0f * sk.branches[size_t(bestBr)].baseR / 1.6f + 0.5f));
                journal::info("рубка", "корень {}: удар {} из {}", bestR + 1, cut.rootHits, need);
                if (cut.rootHits >= need) {
                    cut.rootsCut |= 1u << bestR;
                    cut.rootTarget = -1;
                    best->dirty = true;
                    glm::vec3 corner;
                    const auto& rootOf = cut.rootOf;
                    auto m = game::voxelModel(sk, [&rootOf, bestR](float, float, float, int b) -> int {
                        return b > 0 && b < int(rootOf.size()) && rootOf[size_t(b)] == bestR ? 1 : 0;
                    }, corner);
                    if (m) {
                        const float d = 2.0f * sk.branches[size_t(bestBr)].baseR;
                        game::Item it{game::ItemKind::SNAG, d >= 8.0f ? game::ItemSize::LARGE : (d >= 4.0f ? game::ItemSize::MEDIUM : game::ItemSize::SMALL), m};
                        glm::dvec3 p = (org + glm::dvec3(corner + glm::vec3(m->center) + 0.5f)) / 16.0;
                        p.y = surfaceAbove(p);
                        items.spawn(it, p, glm::quat(1, 0, 0, 0), glm::vec3(0, 1.0f, 0), glm::vec3(0));
                        journal::info("рубка", "корень перерублен: {}", game::itemName(it));
                    }
                }
            } else if (tool == game::ItemKind::SHOVEL && cut.allRootsCut()) {
                spray(wp, -glm::vec3(dir), 6);
                if (++cut.digHits >= 4) {   // пень выкопан - коряга по толщине ствола
                    cut.dug = true;
                    best->dirty = true;
                    glm::vec3 corner;
                    const game::TreeCut cc = cut;
                    auto m = game::voxelModel(sk, [&cc, &sk](float x, float y, float z, int b) -> int {
                        if (b < 0 || cc.inNotch(x, y, z)) return 0;
                        if (sk.branches[size_t(b)].root) return 0;
                        return cc.falls(b, y) ? 0 : 1;
                    }, corner);
                    if (m) {
                        const float d = 2.0f * cut.R;
                        game::Item it{game::ItemKind::SNAG, d >= 16.0f ? game::ItemSize::LARGE : (d >= 8.0f ? game::ItemSize::MEDIUM : game::ItemSize::SMALL), m};
                        glm::dvec3 p = (org + glm::dvec3(corner + glm::vec3(m->center) + 0.5f)) / 16.0;
                        p.y = surfaceAbove(p) + 0.3;
                        items.spawn(it, p, glm::quat(1, 0, 0, 0), glm::vec3(0, 1.5f, 0), glm::vec3(0.4f, 0, 0.2f));
                        journal::info("рубка", "пень выкопан: {}", game::itemName(it));
                    }
                }
            }
            return;
        }
        if (tool != game::ItemKind::AXE && tool != game::ItemKind::SAW) return;
        if (!cut.notched) {
            cut.notched = true;
            cut.y0 = std::max(hitP.y, 6.0f);
            if (!game::trunkAt(sk, cut.y0, cut.center, cut.R)) { cut.center = glm::vec2(0); cut.R = sk.nodes[0].r; }
            const glm::vec2 toMe = glm::vec2(float(start.x * 16.0 - org.x), float(start.z * 16.0 - org.z)) - cut.center;
            cut.dir = glm::length(toMe) > 1e-3f ? glm::normalize(toMe) : glm::vec2(1, 0);
        }
        cut.depth += cut.perHit() * (tool == game::ItemKind::SAW ? 1.5f : 1.0f);
        ++cut.hits;
        cut.leafKeep = std::max(0.25f, cut.leafKeep * 0.85f);   // Р102: листва осыпается с каждым ударом
        dropLeaves((org + glm::dvec3(0, sk.height * 0.65, 0)) / 16.0, sk.crownR / 16.0f * 0.6f, 40);
        best->dirty = true;
        kick = 0.035f;
        spray(wp, glm::vec3(cut.dir.x, 0, cut.dir.y), 14);
        journal::info("рубка", "удар {}: зарубка {:.1f} из {:.1f} мелких вокселей", cut.hits, cut.depth, 2.0f * cut.R * 0.75f);
        if (cut.ready()) {
            // Р93: ДЕРЕВО ПАДАЕТ - от игрока. Падающая часть - телом (выше реза, по скелету), на месте - пень.
            cut.felled = true;
            cut.computeParts(sk);
            FallenTree ft;
            ft.tb = game::makeTreeBody(best->spot.key, best->spot.base, best->spot.shift, sk, cut, -cut.dir);
            fallen.push_back(std::move(ft));
            journal::info("рубка", "дерево падает: {} ударов", cut.hits);
        }
    };
    while (running) {
        for (double& v : sect) v = 0;
        const auto iterAt = clock_t_::now();
        lapAt = iterAt;
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_EVENT_QUIT) running = false;
            // Р73: в режиме снимка мышь и клавиши не трогают камеру - курсор над окном поворачивал её, замеры плавали
            if (shot && ev.type != SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED) continue;
            // T - ввод координат перехода; пока вводим, движение камерой отключено (иначе буквы её тащат)
            if (ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_T && !typing) {
                typing = true;
                gotoText.clear();
                SDL_StartTextInput(window);
                continue;
            }
            if (typing) {
                if (ev.type == SDL_EVENT_TEXT_INPUT) gotoText += ev.text.text;
                if (ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_BACKSPACE && !gotoText.empty()) gotoText.pop_back();
                if (ev.type == SDL_EVENT_KEY_DOWN && (ev.key.key == SDLK_ESCAPE || ev.key.key == SDLK_RETURN)) {
                    glm::vec3 to(0.0f);
                    bool hasY = false;
                    if (ev.key.key == SDLK_RETURN && dev::parseGoto(gotoText, to, hasY)) {
                        if (!hasY) to.y = dev::groundAt(tp, to.x, to.z) + 3.0f;
                        eye = glm::dvec3(to);
                        journal::info("переход", "камера переставлена в ({:.0f}, {:.0f}, {:.0f})", to.x, to.y, to.z);
                    }
                    typing = false;
                    gotoText.clear();
                    SDL_StopTextInput(window);
                }
                continue;
            }
            if (ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_F3) { showPanel = !showPanel; panelLines.clear(); }
            if (ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_ESCAPE) running = false;
            if (ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_L) gpu->setDebug(debug ^= 1);
            if (ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_O) {   // Р53: показательный дуб перед камерой
                const glm::vec2 f = glm::normalize(glm::vec2(std::cos(yaw), std::sin(yaw)));
                const int32_t x = int32_t(std::floor((eye.x + f.x * 2.0f) * 16)), z = int32_t(std::floor((eye.z + f.y * 2.0f) * 16));   // 32 блока
                oakSeed = oakSeed * 1664525u + 1013904223u;
                oaks.push_back({x, oakBase(x, z), z, oakSeed});
                buildOaks();
            }
            // Р83: кнопка мыши (любая) - сломать блок под прицелом. Он уходит в правки мира, затронутые
            // столбцы перестраиваются, и грани соседей, прежде закрытые, открываются сами.
            // Р99: ТЁСКА - мышь водит курсор по клеткам, ЛКМ скалывает; E или Esc - отложить
            // Р99: тёска как в TFC - обычный курсор, камень стоит; клетка - под курсором, ЛКМ (и протяжка) скалывает
            const auto knapAt = [&](float px, float py) {
                const float t35 = std::tan(glm::radians(35.0f)), d = 0.34f, aspect = float(width) / float(std::max(1, height));
                const float nx = 2.0f * px / float(width) - 1.0f, ny = 1.0f - 2.0f * py / float(height);
                const float u = nx * d * t35 * aspect, v = ny * d * t35 + 0.03f;   // от середины камня, блоки
                const float x = u * 64.0f + 8.5f, y = v * 64.0f + 8.5f;            // воксели плиты (17×17)
                const int cx = int(std::floor((x - 1.0f) / 3.0f)), cyb = int(std::floor((y - 1.0f) / 3.0f));
                const bool in = cx >= 0 && cx < 5 && cyb >= 0 && cyb < 5;
                knap->cx = in ? cx : -1;
                knap->cy = in ? 4 - cyb : -1;
            };
            const auto endKnap = [&] { knap.reset(); if (!shot) SDL_SetWindowRelativeMouseMode(window, true); };
            bool chip = false;
            if (knap && ev.type == SDL_EVENT_MOUSE_MOTION) {
                knapAt(ev.motion.x, ev.motion.y);
                chip = knap->down;
            }
            if (knap && ev.type == SDL_EVENT_MOUSE_BUTTON_UP && ev.button.button == SDL_BUTTON_LEFT) { knap->down = false; continue; }
            if (knap && ev.type == SDL_EVENT_MOUSE_BUTTON_DOWN && ev.button.button == SDL_BUTTON_LEFT) {
                knapAt(ev.button.x, ev.button.y);
                knap->down = true;
                chip = true;
            }
            if (knap && (ev.type == SDL_EVENT_MOUSE_MOTION || ev.type == SDL_EVENT_MOUSE_BUTTON_DOWN)) {
                if (chip && knap->cx >= 0 && knap->cell[knap->cy][knap->cx]) {
                    knap->cell[knap->cy][knap->cx] = false;
                    chips.push_back({glm::vec3(eye * 16.0) + glm::vec3(std::cos(yaw), 0, std::sin(yaw)) * 0.35f, glm::vec3(0, 1.0f, 0), 0.6f});
                    const game::ItemKind res = game::knapResult(*knap);
                    if (res != game::ItemKind::STONE) {
                        hands[knap->hand] = game::makeHead(res, knap->rock, *knap);
                        journal::info("тёска", "выточено: {}", game::itemName(*hands[knap->hand]));
                        endKnap();
                    }
                }
                continue;
            }
            if (knap && ev.type == SDL_EVENT_KEY_DOWN && (ev.key.key == SDLK_E || ev.key.key == SDLK_ESCAPE)) { endKnap(); continue; }
            if (!knap && !typing && ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_E) {
                const int h = hands[1] && hands[1]->kind == game::ItemKind::STONE ? 1 : (hands[0] && hands[0]->kind == game::ItemKind::STONE ? 0 : -1);
                if (h >= 0) {
                    knap = game::Knap{};
                    knap->hand = h;
                    knap->rock = hands[h]->mat;
                    knap->cx = knap->cy = -1;
                    if (!shot) { SDL_SetWindowRelativeMouseMode(window, false); SDL_WarpMouseInWindow(window, float(width) * 0.5f, float(height) * 0.5f); }
                }
                continue;
            }
            // Р100: F2 - рамка крафта: появилась (едет за прицелом) -> встала -> убрана
            if (!typing && ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_F2) { craft.stage = (craft.stage + 1) % 3; continue; }
            // Р100: готовое в пятой ячейке - взять пустой рукой; исходные предметы уходят
            if (craft.stage == 2 && ev.type == SDL_EVENT_MOUSE_BUTTON_DOWN && (ev.button.button == SDL_BUTTON_LEFT || ev.button.button == SDL_BUTTON_RIGHT) &&
                !SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_V] && !SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_Q]) {
                const int hand = ev.button.button == SDL_BUTTON_LEFT ? 0 : 1;
                const glm::dvec3 st = eye * 16.0;
                const glm::dvec3 dr(std::cos(pitch) * std::cos(yaw), std::sin(pitch), std::cos(pitch) * std::sin(yaw));
                game::Item out;
                std::string name;
                std::vector<size_t> idx;
                if (!hands[hand] && craftCell(st, dr) == 4 && craftMatch(out, name, idx)) {
                    std::sort(idx.rbegin(), idx.rend());
                    for (size_t k : idx) {
                        for (uint32_t m : items.list[k].meshes) gpu->destroyMesh(m);
                        items.list.erase(items.list.begin() + long(k));
                    }
                    hands[hand] = out;
                    journal::info("крафт", "собрано: {}", name);
                    continue;
                }
            }
            // Р99: зажатая R + клик - второе действие: камнем соскоблить волокно с молодого дерева, топором - окорить
            // лежащий ствол. Зажатая X + клик - метнуть предмет из этой руки.
            if (ev.type == SDL_EVENT_MOUSE_BUTTON_DOWN && (ev.button.button == SDL_BUTTON_LEFT || ev.button.button == SDL_BUTTON_RIGHT) &&
                (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_R] || SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_X] ||
                 (ev.button.clicks >= 2 && hands[ev.button.button == SDL_BUTTON_LEFT ? 0 : 1] && !craft.stage &&
                  handAge[ev.button.button == SDL_BUTTON_LEFT ? 0 : 1] > 0.5f))) {
                const int hand = ev.button.button == SDL_BUTTON_LEFT ? 0 : 1;
                const glm::dvec3 st = eye * 16.0;
                const glm::dvec3 dr(std::cos(pitch) * std::cos(yaw), std::sin(pitch), std::cos(pitch) * std::sin(yaw));
                // Р104: метнуть - двойной щелчок кнопкой этой руки (или X + клик); удар первого щелчка отменяется
                if (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_X] || ev.button.clicks >= 2) {
                    if (pendingChop && pendingChop->hand == hand) pendingChop.reset();
                    if (hands[hand]) {
                        static uint32_t sp = 99u;
                        sp = sp * 1664525u + 1013904223u;
                        const bool tool = hands[hand]->tool();
                        // орудие летит остриём вперёд и вращаясь поперёк; камень - кувырком
                        const glm::vec3 ang = tool ? glm::vec3(-std::sin(yaw), 0, std::cos(yaw)) * 9.0f : glm::vec3(float(sp & 255) / 32.0f - 4.0f, float((sp >> 8) & 255) / 32.0f - 4.0f, 3.0f);
                        items.spawn(*hands[hand], st + dr * 0.6, glm::angleAxis(-yaw, glm::vec3(0, 1, 0)), glm::vec3(dr) * 17.0f + glm::vec3(0, 2.0f, 0), ang);
                        journal::info("руки", "метнул: {}", game::itemName(*hands[hand]));
                        hands[hand].reset();
                        swing[hand] = SWING;
                    }
                    continue;
                }
                if (hands[hand] && (hands[hand]->kind == game::ItemKind::STONE || hands[hand]->kind == game::ItemKind::KNIFE)) {
                    // Р102: КОРА С ЖИВОГО ДЕРЕВА - не без конца. Молодое даёт волокно (лыко), взрослое - кору.
                    // Запас: у молодого 4 пучка (по 3 удара), у взрослого - стружки по толщине ствола (по 2 удара).
                    for (auto& [key, t] : forest) {
                        if (!t.skel) continue;
                        if (treeCuts.count(key) && treeCuts[key].felled) continue;
                        const glm::dvec3 org = glm::dvec3(t.spot.base) * 16.0 + glm::dvec3(t.spot.shift.x, -3, t.spot.shift.z);
                        const glm::dvec3 o = st * 16.0 - org;
                        if (o.x * o.x + o.z * o.z > 64.0 * 64.0) continue;
                        float th;
                        glm::vec3 hp;
                        if (!game::hitTrunk(*t.skel, glm::vec3(o), glm::vec3(dr), 3.0f * 16.0f, th, hp)) continue;
                        swing[hand] = SWING;
                        const bool young = t.spot.age == world::TreeAge::YOUNG;
                        glm::vec2 c;
                        float R = 2;
                        game::trunkAt(*t.skel, 24.0f, c, R);
                        const int per = young ? 3 : 2;
                        const int maxHits = young ? 12 : 2 * std::max(1, int(2.0f * R / 4.0f + 0.5f)) * per;
                        game::TreeCut& cut = treeCuts[key];
                        if (cut.scraped >= maxHits) { journal::info("руки", "кора здесь уже содрана"); break; }
                        ++cut.scraped;
                        cut.leafKeep = std::max(0.25f, cut.leafKeep * 0.97f);
                        t.dirty = true;
                        chips.push_back({glm::vec3((org + glm::dvec3(hp)) / 16.0), -glm::vec3(dr) * 2.0f + glm::vec3(0, 1, 0), 0.8f});
                        if (cut.scraped % per == 0) {
                            items.spawn(young ? game::makeFiber() : game::makeBark(), (org + glm::dvec3(hp)) / 16.0 - dr * 0.3, glm::quat(1, 0, 0, 0),
                                        glm::vec3(0, 1.0f, 0), {});
                            if (young) journal::info("руки", "содрано лыко (волокно), осталось {}", (maxHits - cut.scraped) / per);
                            else journal::info("руки", "содрана кора, осталось {}", (maxHits - cut.scraped) / per);
                        }
                        break;
                    }
                } else if (hands[hand] && hands[hand]->kind == game::ItemKind::AXE) {
                    // топор вторым действием - окорка лежащего ствола (как скобелем, но медленнее не делаем)
                    for (FallenTree& ft : fallen) {
                        if (!ft.tb.free) continue;
                        float tt;
                        glm::vec3 lp;
                        if (game::hitBranch(ft.tb, st, dr, 3.2f, tt, &lp) != 0) continue;
                        swing[hand] = SWING;
                        const int strips = game::debarkBody(ft.tb, lp.y);
                        for (int k = 0; k < strips; ++k)
                            items.spawn(game::makeBark(), st + dr * double(tt) + glm::dvec3(0, 0.3, 0), glm::quat(1, 0, 0, 0), glm::vec3(0, 1.5f, 0), {});
                        break;
                    }
                }
                continue;
            }
            if (ev.type == SDL_EVENT_MOUSE_BUTTON_DOWN &&
                (ev.button.button == SDL_BUTTON_RIGHT || ev.button.button == SDL_BUTTON_LEFT) &&
                !SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_V] && !SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_Q]) {
                const glm::dvec3 start = eye * 16.0;                       // прежние меры -> блоки (Р54)
                const glm::dvec3 dir(std::cos(pitch) * std::cos(yaw), std::sin(pitch), std::cos(pitch) * std::sin(yaw));
                // Р92: ЛКМ - левая рука, ПКМ - правая. Предмет под прицелом (ближе блока) - взять, если рука
                // свободна. Пустая рука по блоку - сломать, как прежде. В руке предмет - его действие (рубка, Р93).
                const int hand = ev.button.button == SDL_BUTTON_LEFT ? 0 : 1;
                double tItem = 0, tBlock = 1e9;
                const int picked = items.pick(start, dir, 4.5, tItem);
                for (int i = 1; i <= 6 * 8; ++i) {
                    const glm::dvec3 v = start + dir * (double(i) / 8.0);
                    // Р94: луч прицела - по рельефу: блоки стволов (осязаемость) кору заслонять не должны
                    if (voxelQuery.solidTerrain(int32_t(std::floor(v.x)), int32_t(std::floor(v.y)), int32_t(std::floor(v.z)))) { tBlock = double(i) / 8.0; break; }
                }
                if (picked >= 0 && tItem < tBlock && hands[hand] && hands[hand]->kind == game::ItemKind::DRAWKNIFE &&
                    items.list[size_t(picked)].item.kind == game::ItemKind::LOG) {
                    // Р96: окорка бревна-предмета скобелем: метр вдоль бревна под прицелом
                    game::WorldItem& w = items.list[size_t(picked)];
                    auto m = std::make_shared<game::ItemModel>(*w.item.model);
                    const glm::vec3 lp = glm::transpose(glm::mat3_cast(w.body.rot)) * glm::vec3(start + dir * tItem - w.body.pos) * float(m->res) +
                                         glm::vec3(m->center) + 0.5f;
                    const int strips = game::debarkItem(*m, lp);
                    if (strips) {
                        m->finish();
                        w.item.model = m;
                        for (uint32_t id : w.meshes) gpu->destroyMesh(id);
                        w.meshes.clear();
                        w.offs.clear();
                        const glm::dvec3 at = start + dir * tItem;
                        for (int k = 0; k < strips; ++k)
                            items.spawn(game::makeBark(), at + glm::dvec3(0, 0.3, 0), glm::angleAxis(float(k) * 1.3f, glm::vec3(0, 1, 0)),
                                        glm::vec3(0.3f * float(k % 3) - 0.3f, 1.5f, 0.2f), glm::vec3(1, 0.5f, 0));
                        journal::info("рубка", "бревно окорено: стружек коры {}", strips);
                    }
                    swing[hand] = SWING;
                } else if (picked >= 0 && tItem < tBlock) {
                    if (!hands[hand]) {
                        game::WorldItem& w = items.list[size_t(picked)];
                        hands[hand] = w.item;
                        if (w.nat) { takenNatural.insert(w.nat); naturalSpawned.erase(w.nat); }   // Р99: природное - не вернётся
                        for (uint32_t m : w.meshes) gpu->destroyMesh(m);
                        journal::info("руки", "взят в {} руку: {}", hand ? "правую" : "левую", game::itemName(w.item));
                        items.wake(w.body.pos, 2.0);
                        items.list.erase(items.list.begin() + picked);
                    }
                } else if (hands[hand] && hands[hand]->tool() && swing[hand] <= 0.15f) {
                    // замах вбок; удар засчитывается в середине прохода, а не в миг нажатия
                    swing[hand] = SWING;
                    pendingChop = PendingChop{hand, start, dir, tBlock, SWING * 0.42f};   // середина прохода
                } else if (!hands[hand])
                for (int i = 1; i <= 6 * 8; ++i) {                          // до 6 блоков, шаг 1/8
                    const glm::dvec3 v = start + dir * (double(i) / 8.0);
                    const int32_t vx = int32_t(std::floor(v.x)), vy = int32_t(std::floor(v.y)), vz = int32_t(std::floor(v.z));
                    const world::Material m = voxelQuery.at(vx, vy, vz);
                    if (m == world::AIR || m == world::WATER) continue;
                    edits.set(vx, vy, vz, world::AIR);
                    // Р99: из глинистой земли - ком глины
                    if (m == world::Material(world::SOIL_FIRST + 6) || m == world::Material(world::SOIL_FIRST + 5))
                        items.spawn(game::makeClay(), glm::dvec3(vx + 0.5, vy + 0.5, vz + 0.5), glm::quat(1, 0, 0, 0), glm::vec3(0, 1.0f, 0), {});
                    const auto fd = [](int32_t a) { return a >= 0 ? a / 32 : -((-a + 31) / 32); };
                    const int32_t cx = fd(vx), cz = fd(vz);
                    // сломали у границы куска - перестроить и соседа: у него открылась грань
                    for (int dz = -1; dz <= 1; ++dz)
                        for (int dx = -1; dx <= 1; ++dx)
                            if (fd(vx + dx) == cx + dx && fd(vz + dz) == cz + dz) voxels.invalidate(cx + dx, cz + dz);
                    voxels.invalidate(cx, cz);
                    journal::info("мир", "сломан блок: {} в ({}, {}, {})", dev::materialName(m), vx, vy, vz);
                    items.wake(glm::dvec3(vx + 0.5, vy + 0.5, vz + 0.5), 4.0);   // Р92: лежавшее на нём падает
                    break;
                }
            }
            if (ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_K) { oaks.clear(); buildOaks(); }
            if (ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_G) {   // Р60: роща в 40 блоках перед камерой
                const glm::vec2 f = glm::normalize(glm::vec2(std::cos(yaw), std::sin(yaw)));
                oakSeed = oakSeed * 1664525u + 1013904223u;
                plantGrove(int32_t(std::floor((eye.x + f.x * 2.5f) * 16)), int32_t(std::floor((eye.z + f.y * 2.5f) * 16)), 25, 40, oakSeed);
                buildOaks();
            }
            if (ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_L) leavesOn = !leavesOn;
            if (ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_F5) saveNow();   // Р98: сохранить сейчас
            // Р92: V - положить к вокселю под прицелом, Q - выбросить. Из правой руки, если она пуста - из левой.
            // Р95: зажатая V + ЛКМ/ПКМ - положить из левой/правой руки; зажатая Q + ЛКМ/ПКМ - выбросить
            if (ev.type == SDL_EVENT_MOUSE_BUTTON_DOWN && !typing && (ev.button.button == SDL_BUTTON_LEFT || ev.button.button == SDL_BUTTON_RIGHT) &&
                (SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_V] || SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_Q])) {
                const bool placeKey = SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_V];
                const int hand = ev.button.button == SDL_BUTTON_LEFT ? (hands[0] ? 0 : -1) : (hands[1] ? 1 : -1);
                if (hand >= 0) {
                    const game::Item it = *hands[hand];
                    const game::ItemModel& md = *it.model;
                    const glm::dvec3 start = eye * 16.0;
                    const glm::dvec3 dir(std::cos(pitch) * std::cos(yaw), std::sin(pitch), std::cos(pitch) * std::sin(yaw));
                    if (placeKey) {
                        glm::ivec3 prev(INT32_MIN);
                        bool hit = false;
                        for (int i = 1; i <= 6 * 8 && !hit; ++i) {
                            const glm::dvec3 v = start + dir * (double(i) / 8.0);
                            const glm::ivec3 b(int(std::floor(v.x)), int(std::floor(v.y)), int(std::floor(v.z)));
                            if (voxelQuery.solid(b.x, b.y, b.z)) hit = true;
                            else prev = b;
                        }
                        if (hit && prev.x != INT32_MIN) {
                            // длинной стороной - вдоль земли, поворот - к игроку, с шагом в четверть круга
                            glm::quat rq(1, 0, 0, 0);
                            if (md.sy >= md.sx && md.sy >= md.sz) rq = glm::angleAxis(1.5707963f, glm::vec3(1, 0, 0));
                            const float ys = std::round(yaw / 1.5707963f) * 1.5707963f;
                            rq = glm::angleAxis(-ys, glm::vec3(0, 1, 0)) * rq;
                            const glm::mat3 R = glm::mat3_cast(rq);
                            float low = 1e9f;
                            for (int c = 0; c < 8; ++c) {
                                const glm::vec3 k(c & 1 ? md.hi.x : md.lo.x, c & 2 ? md.hi.y : md.lo.y, c & 4 ? md.hi.z : md.lo.z);
                                low = std::min(low, (R * k).y);
                            }
                            glm::dvec3 at(prev.x + 0.5, prev.y - low + 0.002, prev.z + 0.5);
                            const int cell = craftCell(start, dir);   // Р100: в рамку крафта - в середину ячейки
                            if (cell >= 0 && cell < 4)
                                at = craft.corner + glm::dvec3((cell % 2) * 0.25 + 0.125, -low + 0.002, (cell / 2) * 0.25 + 0.125);
                            items.spawn(it, at, rq, {}, {}, true);
                            hands[hand].reset();
                        }
                    } else {
                        static uint32_t spin = 12345u;
                        spin = spin * 1664525u + 1013904223u;
                        const glm::vec3 ang(float(spin & 255) / 64.0f - 2.0f, float((spin >> 8) & 255) / 64.0f - 2.0f, float((spin >> 16) & 255) / 64.0f - 2.0f);
                        items.spawn(it, start + dir * 0.7, glm::angleAxis(-yaw, glm::vec3(0, 1, 0)), glm::vec3(dir) * 6.0f + glm::vec3(0, 1.5f, 0), ang);
                        hands[hand].reset();
                    }
                }
            }   // Р91: листва - для проверки зимы
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
        static const bool noKeys[SDL_SCANCODE_COUNT] = {};
        const bool* keys = (shot || typing) ? noKeys : SDL_GetKeyboardState(nullptr);
        // Р94: ОСЯЗАЕМОСТЬ СТВОЛОВ - блоки, через которые проходит ствол или толстый сук (стоящие деревья рядом,
        // пни, поваленные тела). Точность - блок: ствол в 30 см занимает один-два блока, как бревно.
        {
            staticTrunks.clear();
            const auto raster = [](std::unordered_set<uint64_t>& out, glm::dvec3 a, glm::dvec3 b, double r) {   // блоки
                const double len = glm::distance(a, b);
                const int n = std::max(1, int(len / 0.25));
                const double rr = std::max(0.0, r - 0.15);
                for (int k = 0; k <= n; ++k) {
                    const glm::dvec3 p = a + (b - a) * (double(k) / n);
                    for (int z = int(std::floor(p.z - rr)); z <= int(std::floor(p.z + rr)); ++z)
                        for (int y = int(std::floor(p.y - rr)); y <= int(std::floor(p.y + rr)); ++y)
                            for (int x = int(std::floor(p.x - rr)); x <= int(std::floor(p.x + rr)); ++x) out.insert(world::editKey(x, y, z));
                }
            };
            const glm::dvec3 eb = eye * 16.0;
            for (auto& [key, t] : forest) {
                if (!t.skel) continue;
                const glm::dvec3 org = glm::dvec3(t.spot.base) * 16.0 + glm::dvec3(t.spot.shift.x, -3, t.spot.shift.z);
                if (glm::distance(glm::dvec2(org.x, org.z) / 16.0, glm::dvec2(eb.x, eb.z)) > 14.0) continue;
                const auto ci = treeCuts.find(key);
                const game::TreeCut* cut = ci != treeCuts.end() ? &ci->second : nullptr;
                if (cut && cut->dug) continue;
                const world::TreeSkel& sk = *t.skel;
                for (size_t i = 1; i < sk.nodes.size(); ++i) {
                    const world::TreeNode& n = sk.nodes[i];
                    const world::TreeNode& pa = sk.nodes[size_t(n.parent)];
                    if (sk.branches[size_t(n.branch)].root) continue;
                    if (n.branch != 0) continue;   // твёрд только ствол: сучья не должны отгораживать дерево
                    if (cut && cut->felled && (n.branch != 0 || pa.p[1] >= cut->y0)) continue;   // от срубленного - только пень
                    glm::vec3 b(n.p[0], n.p[1], n.p[2]);
                    if (cut && cut->felled && b.y > cut->y0) b.y = cut->y0;
                    raster(staticTrunks, (org + glm::dvec3(pa.p[0], pa.p[1], pa.p[2])) / 16.0, (org + glm::dvec3(b)) / 16.0,
                           double(std::max(pa.r, n.r)) / 16.0);
                }
            }
            allTrunks = staticTrunks;
            for (const FallenTree& ft : fallen) {
                const game::TreeBody& tb = ft.tb;
                const world::TreeSkel& sk = *tb.skel;
                for (size_t i = 1; i < sk.nodes.size(); ++i) {
                    const world::TreeNode& n = sk.nodes[i];
                    if (!tb.owns(n.branch, n.p[1]) || n.r <= 0 || n.branch != 0) continue;   // только ствол - на него можно встать
                    const world::TreeNode& pa = sk.nodes[size_t(n.parent)];
                    raster(allTrunks, tb.toWorld(glm::vec3(pa.p[0], std::max(pa.p[1], n.branch == 0 ? tb.cut.y0 : pa.p[1]), pa.p[2])),
                           tb.toWorld(glm::vec3(n.p[0], n.p[1], n.p[2])), double(n.r) / 16.0);
                }
            }
            voxelQuery.extra = &allTrunks;
        }
        // Р83: движение - через тело игрока (player.h). В полёте всё как прежде, пешком - притяжение,
        // упоры в блоки, прыжок и бег; двойной прыжок переключает одно в другое.
        {
            game::Input in;
            in.forward = float(keys[SDL_SCANCODE_W]) - float(keys[SDL_SCANCODE_S]);
            in.strafe = float(keys[SDL_SCANCODE_D]) - float(keys[SDL_SCANCODE_A]);
            in.jumpHeld = keys[SDL_SCANCODE_SPACE];
            in.jumpPressed = in.jumpHeld && !jumpWas;
            jumpWas = in.jumpHeld;
            in.sprint = keys[SDL_SCANCODE_LCTRL];    // Р95: бег - Ctrl
            in.crouch = keys[SDL_SCANCODE_LSHIFT];   // присесть - Shift
            in.prone = keys[SDL_SCANCODE_C];         // лечь - C
            player.eye = eye;
            player.flySpeed = speedScale;
            game::step(player, voxelQuery, in, yaw, dt, std::chrono::duration<float>(now - startAt).count());
            eye = player.eye;
        }
        // Р92: на старте рядом с игроком - инструменты (падают на землю сами)
        if (!toolsSpawned) {
            toolsSpawned = true;
            const glm::dvec3 f = glm::normalize(glm::dvec3(fwd.x, 0, fwd.z)), sd(-f.z, 0, f.x);
            const game::ItemKind kinds[4] = {game::ItemKind::AXE, game::ItemKind::SAW, game::ItemKind::DRAWKNIFE, game::ItemKind::SHOVEL};
            for (int k = 0; k < 4 && (giveTools || autoChop); ++k)
                items.spawn(game::makeTool(kinds[k]), eye * 16.0 + f * 3.0 + sd * (double(k) - 1.5) * 0.9 - glm::dvec3(0, 0.6, 0),
                            glm::angleAxis(1.5707963f, glm::vec3(1, 0, 0)), {}, glm::vec3(0.3f * k, 0, 0.2f));
        }
        items.step(voxelQuery, dt, eye * 16.0, 96.0);
        // Р99: ПРИРОДНОЕ - камушки (порода под ногами) и сушняк (в лесу) лежат вокруг игрока; дальше 48 блоков -
        // убираются, подобранное помнится и не возвращается
        naturalAt += dt;
        if (naturalAt > 0.5f) {
            naturalAt = 0;
            const glm::ivec3 eb(int(std::floor(eye.x * 16.0)), int(std::floor(eye.y * 16.0)), int(std::floor(eye.z * 16.0)));
            for (size_t k = items.list.size(); k-- > 0;) {
                game::WorldItem& w = items.list[k];
                if (!w.nat || glm::distance(glm::dvec2(w.body.pos.x, w.body.pos.z), glm::dvec2(eb.x, eb.z)) < 48.0) continue;
                for (uint32_t m : w.meshes) gpu->destroyMesh(m);
                naturalSpawned.erase(w.nat);
                items.list.erase(items.list.begin() + long(k));
            }
            constexpr int CELL = 4, RAD = 36;
            int made = 0;
            for (int cz = (eb.z - RAD) / CELL - 1; cz <= (eb.z + RAD) / CELL + 1 && made < 40; ++cz)
                for (int cx = (eb.x - RAD) / CELL - 1; cx <= (eb.x + RAD) / CELL + 1 && made < 40; ++cx) {
                    const int x = cx * CELL, z = cz * CELL;
                    if ((x - eb.x) * (x - eb.x) + (z - eb.z) * (z - eb.z) > RAD * RAD) continue;
                    const uint64_t key = (uint64_t(uint32_t(cx)) << 32) ^ uint32_t(cz) ^ 0x5701E00000000000ull;
                    if (takenNatural.count(key) || naturalSpawned.count(key)) continue;
                    uint32_t h = uint32_t(cx) * 0x8DA6B343u ^ uint32_t(cz) * 0xD8163841u ^ tp.seed * 0x9E3779B1u ^ 0x5701Eu;
                    h ^= h >> 15; h *= 0x2C1B3C6Du; h ^= h >> 12; h *= 0x297A2D39u; h ^= h >> 15;
                    const float u = float(h & 0xFFFF) / 65536.0f;
                    if (u >= 0.32f) { naturalSpawned[key] = 0; continue; }
                    const int px = x + int((h >> 16) % CELL), pz = z + int((h >> 20) % CELL);
                    int32_t top = INT32_MIN;
                    for (int y = eb.y + 40; y > eb.y - 60; --y)
                        if (voxelQuery.solidTerrain(px, y, pz)) { top = y; break; }
                    if (top == INT32_MIN) continue;   // столбец ещё далеко - позже
                    if (voxelQuery.at(px, top + 1, pz) == world::WATER) { naturalSpawned[key] = 0; continue; }
                    game::Item it;
                    glm::quat rq(1, 0, 0, 0);
                    if (u < 0.10f) {   // камень - из породы под землёй этого места
                        world::Material rock = world::ROCK_FIRST;
                        for (int y = top; y > top - 70; --y) {
                            const world::Material m = voxelQuery.at(px, y, pz);
                            if (m >= world::ROCK_FIRST && m <= world::ROCK_LAST) { rock = m; break; }
                        }
                        it = game::makeStone(rock, h);
                        rq = glm::angleAxis(float(h >> 8 & 255) * 0.0246f, glm::vec3(0, 1, 0));
                    } else {           // сушняк - только в лесу
                        if (world::forestDensity(tp, px, pz, top) < 0.12f) { naturalSpawned[key] = 0; continue; }
                        it = game::makeStick(h);
                        rq = glm::angleAxis(float(h >> 8 & 255) * 0.0246f, glm::vec3(0, 1, 0)) * glm::angleAxis(1.5708f, glm::vec3(1, 0, 0));
                    }
                    const glm::mat3 R = glm::mat3_cast(rq);
                    float low = 1e9f;
                    for (int c = 0; c < 8; ++c) {
                        const glm::vec3 k(c & 1 ? it.model->hi.x : it.model->lo.x, c & 2 ? it.model->hi.y : it.model->lo.y, c & 4 ? it.model->hi.z : it.model->lo.z);
                        low = std::min(low, (R * k).y);
                    }
                    const glm::dvec3 pos(px + 0.2 + 0.6 * double((h >> 24) & 15) / 15.0, top + 1.0 - low + 0.003, pz + 0.2 + 0.6 * double((h >> 28) & 15) / 15.0);
                    const uint32_t id = items.spawn(it, pos, rq, {}, {}, false);
                    items.list.back().nat = key;
                    items.list.back().body.asleep = true;
                    naturalSpawned[key] = id;
                    ++made;
                }
        }
        sinceSave += dt;
        if (sinceSave > 60.0f) { sinceSave = 0; saveNow(); }   // Р98: раз в минуту
        if (autoChop && warm) {   // Р93: проверка без человека - удар раз в 0.5 с по ближайшему дереву с сеткой
            static float acc = 0;
            static uint64_t target = 0;
            acc += dt;
            if (acc > 0.5f) {
                acc = 0;
                if (!target) {
                    double bd = 1e18;
                    for (auto& [key, t] : forest) {
                        if (!t.skel || t.spot.age != world::TreeAge::ADULT) continue;
                        const double d = glm::distance(glm::dvec2(t.spot.base.x, t.spot.base.z), glm::dvec2(eye.x * 16.0, eye.z * 16.0));
                        if (d > 8 && d < bd) { bd = d; target = key; }
                    }
                }
                auto it = forest.find(target);
                if (it != forest.end() && !(treeCuts.count(target) && treeCuts[target].felled)) {
                    const ForestTree& t = it->second;
                    const glm::dvec3 org = glm::dvec3(t.spot.base) * 16.0 + glm::dvec3(t.spot.shift.x, -3, t.spot.shift.z);
                    const glm::dvec3 axis = (org + glm::dvec3(t.skel->nodes[0].p[0], 20.0, t.skel->nodes[0].p[2])) / 16.0;
                    glm::dvec3 from(eye.x * 16.0, axis.y, eye.z * 16.0);
                    const glm::dvec3 d = glm::normalize(axis - from);
                    yaw = float(std::atan2(d.z, d.x));   // камера - на дерево
                    pitch = 0.25f;
                    if (!treeCuts.count(target)) eye = (axis - d * 6.0 + glm::dvec3(0, 0.4, 0)) / 16.0;   // у земли, в 6 блоках
                    from = axis - d * 2.0;
                    chopTree(1, from, d, 1e9);
                }
            }
        }
        // Р93: падение стволов, щепки, замах
        // Р94: поваленные деревья - твёрдые тела; опора - рельеф и неподвижные стволы (пни), без самих тел
        for (FallenTree& ft : fallen) {
            const auto solidStatic = [&](int32_t x, int32_t y, int32_t z) {
                return voxelQuery.solidTerrain(x, y, z) || staticTrunks.count(world::editKey(x, y, z)) > 0;
            };
            const bool wasAsleep = ft.tb.body.asleep;
            {   // отладка: состояние тела раз в секунду
                static float dbg = 0;
                dbg += dt;
                if (dbg > 1.0f && !ft.tb.body.asleep) {
                    dbg = 0;
                    journal::info("рубка", "тело: свободно {}, сон {}, высота {:.2f}, скорость {:.2f}, вращение {:.2f}, точек {}, масса {:.0f}",
                                  ft.tb.free, ft.tb.body.asleep, ft.tb.body.pos.y, glm::length(ft.tb.body.vel), glm::length(ft.tb.body.ang),
                                  ft.tb.probes.size(), ft.tb.mass);
                }
            }
            game::stepTreeBody(ft.tb, solidStatic, dt);
            // легло (или улеглось заново после переката) - ветви сминаются по земле
            if (ft.tb.free && ft.tb.body.asleep && !wasAsleep && !ft.tb.crushed) {
                // Р103: дерево ударилось о землю - большая часть листвы осыпается и не мешает на земле
                ft.tb.cut.leafKeep = std::max(0.15f, ft.tb.cut.leafKeep * 0.35f);
                const world::TreeSkel& sk = *ft.tb.skel;
                dropLeaves(ft.tb.body.pos + glm::dvec3(0, 0.5, 0), sk.crownR / 16.0f * 0.7f, 120);
            }
            if (ft.tb.free && ft.tb.body.asleep && !wasAsleep) {
                game::crushTreeBody(ft.tb, solidStatic);
                journal::info("рубка", "дерево легло, ветви смяты по земле");
            }
            // сетка тела - заново после смятия и обрубки
            if (ft.tb.meshDirty) {
                ft.tb.meshDirty = false;
                std::vector<OakChunk> ch;
                const game::TreeBody& tb = ft.tb;
                buildTreeChunks(*tb.skel, tb.base, tb.shift, [](int32_t, int32_t) { return INT32_MIN; }, ch, 1, leavesOn,
                                [&tb](float x, float y, float z, int b) -> int {
                                    if (tb.cut.inNotch(x, y, z) || !tb.owns(b, y)) return 0;
                                    return b == 0 && tb.debarkedAt(y) ? 2 : 1;   // окорено - древесина
                                }, &tb.cut);
                for (render::MeshId m : ft.meshes) gpu->destroyMesh(m);
                ft.meshes.clear(); ft.offs.clear(); ft.scales.clear();
                for (const OakChunk& oc : ch) { ft.meshes.push_back(gpu->uploadChunkMesh(oc.verts, 0)); ft.offs.push_back(oc.fineOff); ft.scales.push_back(oc.scale); }
            }
        }
        for (Chip& c : chips) {
            if (c.kind == 6) {   // лист планирует: тяжесть слабая, воздух держит, качается
                c.v.y -= 3.0f * dt;
                c.v *= 1.0f - std::min(1.0f, dt * 2.5f);
                c.v.x += std::sin(c.life * 5.0f + c.p.y) * dt * 2.0f;
            } else {
                c.v.y -= 28.0f * dt;
                c.v *= 1.0f - std::min(1.0f, dt * 0.8f);
            }
            c.p += c.v * dt;
            c.life -= dt;
        }
        std::erase_if(chips, [](const Chip& c) { return c.life <= 0; });
        for (float& sw : swing) sw = std::max(0.0f, sw - dt);
        kick = std::max(0.0f, kick - dt * 0.25f);
        if (pendingChop) {
            pendingChop->t -= dt;
            if (pendingChop->t <= 0) { chopTree(pendingChop->hand, pendingChop->start, pendingChop->dir, pendingChop->tBlock); pendingChop.reset(); }
        }
        // Р94: контур вырубки - у сука лежащего дерева под прицелом, если в руке топор
        {
            const auto has = [&](game::ItemKind k) { return (hands[0] && hands[0]->kind == k) || (hands[1] && hands[1]->kind == k); };
            const bool axe = has(game::ItemKind::AXE), saw = has(game::ItemKind::SAW);
            FallenTree* bf = nullptr;
            int bb = -1;
            glm::vec3 lp(0);
            if (axe || saw) {
                const glm::dvec3 st = eye * 16.0;
                const glm::dvec3 dr(std::cos(pitch) * std::cos(yaw), std::sin(pitch), std::cos(pitch) * std::sin(yaw));
                float bt = 3.2f;
                for (FallenTree& ft : fallen) {
                    if (!ft.tb.free) continue;
                    float tt;
                    glm::vec3 l;
                    const int b = game::hitBranch(ft.tb, st, dr, bt, tt, &l);
                    if (b >= 0 && tt < bt) { bt = tt; bb = b; bf = &ft; lp = l; }
                }
            }
            // ключ контура: сук - его номер; отметка раскряжёвки на стволе - 100000 + высота
            float markY = -1;
            if (bf && bb == 0) markY = bf->tb.yLo + std::round((lp.y - bf->tb.yLo) / 16.0f) * 16.0f;
            for (FallenTree& ft : fallen) {
                int want = -1;
                if (&ft == bf && bb > 0 && axe) want = bb;
                if (&ft == bf && bb == 0 && markY > ft.tb.yLo + 8 && markY < ft.tb.yHi - 8) want = 100000 + int(markY);
                if (ft.contour == want) continue;
                for (render::MeshId m : ft.cMeshes) gpu->destroyMesh(m);
                ft.cMeshes.clear(); ft.cOffs.clear(); ft.cModel = nullptr; ft.cHold.reset();
                ft.contour = want;
                if (want < 0) continue;
                ft.cHold = want >= 100000 ? game::trunkContour(ft.tb, float(want - 100000), ft.cCorner) : game::limbContour(ft.tb, want, ft.cCorner);
                if (!ft.cHold) continue;
                ft.cModel = ft.cHold.get();
                std::vector<game::ItemMeshChunk> ch;
                game::meshItem(*ft.cHold, ch);
                for (const game::ItemMeshChunk& c : ch) { ft.cMeshes.push_back(gpu->uploadChunkMesh(c.verts, 0)); ft.cOffs.push_back(c.off); }
            }
        }
        // Р92: шаг для покачивания предмета в руке - по пройденному пешком пути
        {
            handTime += dt;
            for (int h = 0; h < 2; ++h) handAge[h] = hands[h] ? handAge[h] + dt : 0.0f;
            const float hs = glm::length(glm::vec2(player.vel.x, player.vel.z));
            const bool walking = !player.flying && player.onGround && hs > 0.5f;
            walkPhase += hs * dt * 1.9f;
            walkAmp += ((walking ? std::min(1.0f, hs / 4.3f) : 0.0f) - walkAmp) * std::min(1.0f, dt * 6.0f);
        }
        if (shot && warm && spinRate != 0) yaw += spinRate * dt;
        if (shot && warm && flySpeed > 0) eye += glm::dvec3(glm::normalize(glm::vec3(fwd.x, 0, fwd.z))) * double(flySpeed * dt);

        // обратная глубина (Р20): ближняя и дальняя плоскости переставлены местами
        const float farPlane = float(lod.drawDistance) * 1.5f;
        const glm::mat4 proj = glm::perspectiveRH_ZO(glm::radians(70.0f), float(width) / float(std::max(1, height)), farPlane, 0.002f);   // Р56: ближняя плоскость - 3 см (вплотную к коре и листьям)
        const glm::vec3 eyeAbs = eyeF();          // мировое положение одинарной точностью - для отсечения
        // Р93: отдача удара - взгляд на миг вздрагивает
        const float kp = kick > 0 ? kick * std::sin(kick * 400.0f) : 0.0f;
        const glm::vec3 fwdV = glm::normalize(fwd + glm::vec3(0, kp, 0) + right * (kp * 0.5f));
        const glm::mat4 viewProj = proj * glm::lookAt(eyeAbs, eyeAbs + fwdV, glm::vec3(0, 1, 0));
        const Frustum fr(viewProj);
        // Р69: видеокарте - камера относительно начала у куска камеры (целые воксели): малые числа - без дрожи
        const glm::ivec3 originV(int(std::floor(eye.x * 0.5)) * 32, int(std::floor(eye.y * 0.5)) * 32, int(std::floor(eye.z * 0.5)) * 32);
        const glm::dvec3 originOld = glm::dvec3(originV) / 16.0;
        // разность берём в двойной точности и только потом сводим к одинарной: она мала (меньше двух
        // блоков), поэтому у неё остаётся вся точность, и мир перестаёт дёргаться под камерой
        const glm::vec3 eyeLocal = glm::vec3(eye - originOld);
        const glm::mat4 viewProjGpu = proj * glm::lookAt(eyeLocal, eyeLocal + fwd, glm::vec3(0, 1, 0));
        lap(0);
        const auto tUpd = clock_t_::now();
        voxels.update(eye, fwd, fr, 4.0);   // выгрузка готовых столбцов - не дольше 4 мс за кадр
        part[0] = std::max(part[0], msSince(tUpd));
        lap(1);

        {
            const glm::ivec2 want((int(std::floor(eye.x)) - FLOW_SIZE / 2) & ~31, (int(std::floor(eye.z)) - FLOW_SIZE / 2) & ~31);
            if (!flowJob.valid() && (std::abs(want.x - flowAt.x) > FLOW_SIZE / 4 || std::abs(want.y - flowAt.y) > FLOW_SIZE / 4)) {
                flowPending = want;
                flowJob = zoneJobs.submit([&tp, want] {
                    FlowData f;
                    world::flowField(tp, want.x, want.y, FLOW_SIZE, f.vel, &f.ground, &f.surface);
                    return f;
                });
            }
            if (firstFrame && flowJob.valid()) flowJob.wait();   // Р66: лес с первого кадра - нужна земля поля
            if (flowJob.valid() && flowJob.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                FlowData f = take(flowJob, "поле течений");
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
        lap(2);
        // Р75 (вместо Р59/Р61): брызги, пыль и падающая вода - по физике и на всю дальность видимости.
        //   Вблизи (зона мелкой воды, 128 блоков) - кромки: мокрый блок, у которого МОКРЫЙ сосед ниже по глади (вода
        //   падает в воду; к сухому берегу не падает - раньше масса срывалась вдоль берегов лентой). С кромки по всей
        //   её длине срываются блоки воды со скоростью течения поперёк кромки (поле рек: автомат воды блуждал и
        //   менялся с положением камеры - менялось и направление), летят по параболе; где блок уходит под гладь -
        //   всплеск: капли вверх ~0.35·√(2g·h) (h - высота падения), облако пыли, сносимое течением.
        //   Дальше (поле течений, до 2 км) - по клеткам 16 блоков у перепада или быстрой воды: клубы пыли на кромке
        //   (ниже по течению), до 800 блоков - и падающие блоки; размер растёт с расстоянием (не меньше ~3 пикселей),
        //   число - падает. Ещё дальше (плитки, вся дальность) - места падения из плиток: клубы пыли.
        if (!flowGround.empty()) {
            constexpr int T = FINE + 2;
            constexpr float U = 1.0f / 16.0f;
            const std::vector<float>& vel = fieldVel;
            {   // задание кромок: квадрат 32×32 куска вокруг куска камеры
                const auto fd32 = [](int v) { return v >= 0 ? v / 32 : -((-v + 31) / 32); };
                const glm::ivec2 eb(int(std::floor(eye.x * 16.0f)), int(std::floor(eye.z * 16.0f)));
                const glm::ivec2 want((fd32(eb.x) - 16) * 32, (fd32(eb.y) - 16) * 32);
                // после первого кадра: на всех ядрах кромки отнимали их у зоны леса, которую ждёт первый кадр
                if (!firstFrame && !lipJob.valid() && (std::abs(want.x - lipAt.x) >= 256 || std::abs(want.y - lipAt.y) >= 256)) {
                    lipAt = want;
                    lipJob = effectJobs.submit([&tp, want] {
                        const auto t0 = clock_t_::now();
                        auto L = std::make_unique<LipData>();
                        L->origin = want - glm::ivec2(32);
                        L->top.assign(size_t(LIPG) * LIPG, 0.0f);
                        L->wet.assign(size_t(LIPG) * LIPG, 0);
                        // 34×34 столбца кусков - на все ядра (в один поток кромки приходили через ~20 с)
                        std::atomic<int> nextRow{0};
                        const auto rows = [&] {
                            world::ColumnHeights hm;
                            for (int cz; (cz = nextRow++) < LIPG / 32;)
                                for (int cx = 0; cx < LIPG / 32; ++cx) {
                                    world::columnHeights(tp, L->origin.x / 32 + cx, L->origin.y / 32 + cz, hm);
                                    for (int j = 0; j < 32; ++j)
                                        for (int i = 0; i < 32; ++i) {
                                            const size_t k = size_t(cz * 32 + j) * LIPG + cx * 32 + i;
                                            const int32_t h = hm.h[j + 1][i + 1], w = hm.water[j + 1][i + 1];
                                            L->wet[k] = w > h;
                                            L->top[k] = float((w > h ? w : h) + 1);
                                        }
                                }
                        };
                        std::vector<std::thread> th;
                        for (unsigned t = 0; t < std::max(2u, std::thread::hardware_concurrency()); ++t) th.emplace_back(rows);
                        for (auto& t : th) t.join();
                        for (int j = 32; j < LIPG - 32; ++j)
                            for (int i = 32; i < LIPG - 32; ++i) {
                                const size_t k = size_t(j) * LIPG + i;
                                if (!L->wet[k]) continue;
                                for (const auto [dx, dz] : {std::pair{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
                                    const size_t kn = size_t(j + dz) * LIPG + i + dx;
                                    if (!L->wet[kn]) continue;   // к сухому берегу вода не падает
                                    const float h = L->top[k] - L->top[kn];
                                    if (h >= 0.1f)
                                        L->lips.insert(L->lips.end(), {float(L->origin.x + i), float(L->origin.y + j), L->top[k], h, float(dx), float(dz)});
                                }
                            }
                        journal::info("вода", "кромки: {} за {:.0f} мс", L->lips.size() / 6, msSince(t0));
                        return L;
                    });
                }
                if (lipJob.valid() && lipJob.wait_for(std::chrono::seconds(0)) == std::future_status::ready) lipData = take(lipJob, "кромки");
            }
            const LipData* lp = lipData.get();
            // верх глади (у сухого - земли) в блоке, блоки; вне зоны кромок - нет
            const auto topAt = [&](int bx, int bz) {
                if (!lp) return NONE;
                const int i = bx - lp->origin.x, j = bz - lp->origin.y;
                return i < 0 || j < 0 || i >= LIPG || j >= LIPG ? NONE : lp->top[size_t(j) * LIPG + i];
            };
            const auto flowOf = [&](int bx, int bz) {
                const int fx = (bx >> 4) - flowAt.x, fz = (bz >> 4) - flowAt.y;
                if (vel.empty() || fx < 0 || fz < 0 || fx >= FLOW_SIZE || fz >= FLOW_SIZE) return glm::vec2(0);
                return glm::vec2(vel[(size_t(fz) * FLOW_SIZE + fx) * 2], vel[(size_t(fz) * FLOW_SIZE + fx) * 2 + 1]);
            };
            constexpr size_t NEAR_CAP = 60000, ALL_CAP = 100000;
            const auto sprayStep = [&](const float hdt) {   // Р75: шаг брызг (кадр или прогрев)
            float demand = 0.0f;
            const auto spawnUpTo = [&](size_t cap, float rate, float life, auto make) {
                demand += rate * life;
                for (float k = rate * dropRate * hdt; k > 0 && drops.size() < cap; k -= 1.0f) {
                    if (k < 1.0f && rnd() >= k) break;
                    drops.push_back(make());
                }
            };
            const auto spawn = [&](float rate, float life, auto make) { spawnUpTo(NEAR_CAP, rate, life, make); };
            if (lp)
                for (size_t q = 0; q + 6 <= lp->lips.size(); q += 6) {
                    const float* L = &lp->lips[q];
                    const int bx = int(L[0]), bz = int(L[1]);
                    const float v0 = L[2], h = L[3];   // верх глади на кромке и высота падения, блоки
                    const glm::vec2 n{L[4], L[5]}, side{-L[5], L[4]};
                    // кромка - чуть за краем блока, над нижним соседом
                    const glm::vec3 edge(float(bx) + 0.5f + 0.55f * n.x, v0 + 0.02f, float(bz) + 0.5f + 0.55f * n.y);
                    const glm::vec3 eo = edge * U;
                    if (!fr.visible(eo - glm::vec3(U, h * U + U, U), eo + glm::vec3(U))) continue;
                    // вдали блок воды меньше пикселя: крупнее и реже (на 120 блоках - вдвое)
                    const float dist = glm::distance(eo, eyeF()) * 16.0f, scale = std::max(1.0f, dist / 120.0f);
                    const glm::vec2 fv = flowOf(bx, bz);
                    const float sp = glm::length(fv), discharge = (0.6f + sp) / scale;
                    // вылет: поперёк кромки - со скоростью течения (не меньше 0.6 м/с), вдоль - половина его доли
                    const glm::vec2 hv = n * std::max(glm::dot(fv, n), 0.6f) + side * (glm::dot(fv, side) * 0.5f);
                    // блоки воды по всей длине кромки: с водопада (от 3 блоков) - синие, с порога и ступеньки - белые
                    // клочья пены (на малом перепаде вода взбивается, сплошной синей массой не падает)
                    if (h >= 0.5f)
                        spawn((h >= 3.0f ? 4.0f : 3.0f) * discharge, std::sqrt(2.0f * h / 9.8f) + 0.3f, [&] {
                            const float a = rnd() - 0.5f;
                            const glm::vec3 p0 = edge + glm::vec3(side.x * a, -0.1f * rnd(), side.y * a);
                            return Drop{p0 * U, glm::vec3(hv.x * (0.85f + 0.3f * rnd()), 0.15f * rnd(), hv.y * (0.85f + 0.3f * rnd())) * U,
                                        4.0f, h >= 3.0f ? 2 : 3, (h >= 3.0f ? 6.0f + 6.0f * rnd() : 3.0f + 4.0f * rnd()) * scale, p0.y * U};
                        });
                    if (scale < 3.0f)
                        spawn(std::min(h, 3.0f) * 5.0f * discharge, 1.1f, [&] {   // мелкие капли с кромки
                            const float a = rnd() - 0.5f;
                            const glm::vec3 p0 = edge + glm::vec3(side.x * a, 0.05f, side.y * a);
                            return Drop{p0 * U, glm::vec3(hv.x + rnd() - 0.5f, 0.3f + 0.8f * rnd(), hv.y + rnd() - 0.5f) * U,
                                        0.8f + 0.6f * rnd(), 0, (1.0f + rnd()) * scale, p0.y * U};
                        });
                }
            // дальше: клетки поля течений (16 блоков) вне зоны мелкой воды - на всём поле (~2 км)
            for (int z = 0; z < FLOW_SIZE - 1; ++z)
                for (int x = 0; x < FLOW_SIZE - 1; ++x) {
                    const size_t k = size_t(z) * FLOW_SIZE + x;
                    const float sf = flowSurface[k];
                    if (sf <= flowGround[k] + 0.01f) continue;
                    const int bx = (flowAt.x + x) * 16, bz = (flowAt.y + z) * 16;
                    if (lp && bx + 16 > lp->origin.x + 32 && bz + 16 > lp->origin.y + 32 && bx < lp->origin.x + 32 + LIPS && bz < lp->origin.y + 32 + LIPS) continue;
                    if (vel.empty()) continue;
                    const float vx = vel[k * 2], vz = vel[k * 2 + 1], sp = std::sqrt(vx * vx + vz * vz);
                    float drop = 0;   // перепад к соседу ниже по течению, блоки
                    glm::vec2 dir(0);
                    if (sp > 0.01f) {
                        dir = glm::vec2(vx, vz) / sp;
                        const int nx = x + int(std::round(dir.x)), nz = z + int(std::round(dir.y));
                        if (nx >= 0 && nz >= 0 && nx < FLOW_SIZE && nz < FLOW_SIZE) {
                            const size_t kn = size_t(nz) * FLOW_SIZE + nx;
                            if (flowSurface[kn] > flowGround[kn]) drop = std::max(0.0f, (sf - flowSurface[kn]) * 16.0f);
                        }
                    }
                    if (drop < 3.0f && sp < 3.5f) continue;
                    // вода клетки взята в её центре: русло идёт через центр по течению. Пыль - в любом месте этой линии
                    // в пределах клетки (±1.5 блока поперёк), а не у одной точки кромки - иначе ряды через 16 блоков
                    const glm::vec2 side(-dir.y, dir.x);
                    const glm::vec3 c(float(flowAt.x + x) + 0.5f, sf, float(flowAt.y + z) + 0.5f);
                    if (!fr.visible(c - glm::vec3(1.0f, 0.5f, 1.0f), c + glm::vec3(1.0f, 1.0f, 1.0f))) continue;
                    const float dist = glm::distance(glm::vec2(c.x, c.z), glm::vec2(eye.x, eye.z)) * 16.0f;   // блоки
                    const float scale = std::max(1.0f, dist / 600.0f);   // не меньше ~3 пикселей
                    spawn((0.5f * drop + 0.3f * std::max(0.0f, sp - 3.0f)) / scale, 3.5f, [&] {
                        const float a = rnd() - 0.5f, b = (rnd() - 0.5f) * 3.0f * U;
                        return Drop{c + glm::vec3(dir.x * a + side.x * b, (0.2f + rnd() * 1.5f) * U * scale, dir.y * a + side.y * b),
                                    glm::vec3((rnd() - 0.5f) * 1.5f + vx * 0.4f, 0.8f + 1.5f * rnd(), (rnd() - 0.5f) * 1.5f + vz * 0.4f) * U,
                                    2.5f + 2.0f * rnd(), 1, (24.0f + 40.0f * rnd()) * scale, c.y};
                    });
                }
            // ещё дальше - места падения из дальних плиток, вся дальность видимости
            {
                const float cx = float(flowAt.x) + FLOW_SIZE * 0.5f, cz = float(flowAt.y) + FLOW_SIZE * 0.5f, half = FLOW_SIZE * 0.5f - 1.0f;
                farTerrain.forEachFall([&](const float* f) {
                    const glm::vec3 c(f[0] * U, f[1] * U, f[2] * U);
                    if (std::abs(c.x - cx) < half && std::abs(c.z - cz) < half) return;   // в поле течений - клетками
                    const float dist = glm::distance(glm::vec2(c.x, c.z), glm::vec2(eye.x, eye.z)) * 16.0f;
                    if (dist > float(lod.drawDistance) * 16.0f) return;
                    const float scale = std::max(1.0f, dist / 600.0f), big = std::clamp(f[3] / 6.0f, 1.0f, 4.0f);
                    if (!fr.visible(c - glm::vec3(0.5f), c + glm::vec3(0.5f, 0.5f + scale * 0.2f, 0.5f))) return;
                    spawnUpTo(ALL_CAP, 0.4f * big / std::sqrt(scale), 7.0f, [&] {   // выше падение - больше клубов (не крупнее)
                        return Drop{c + glm::vec3((rnd() - 0.5f) * 0.5f, 0.0f, (rnd() - 0.5f) * 0.5f) * (scale * 0.25f),
                                    glm::vec3((rnd() - 0.5f) * 1.5f + f[4] * 1.5f, 1.0f + 1.5f * rnd(), (rnd() - 0.5f) * 1.5f + f[5] * 1.5f) * U * std::sqrt(scale),
                                    5.0f + 4.0f * rnd(), 1, (24.0f + 40.0f * rnd()) * scale, c.y};
                    });
                });
            }
            std::vector<Drop> born;   // всплески - после прохода (вектор капель не растёт под ногами)
            for (Drop& d : drops) {
                if (d.kind != 1) d.v.y -= 9.8f * U * hdt;
                else d.v *= 1.0f - 0.5f * hdt;   // пыль тормозит воздухом
                d.p += d.v * hdt;
                d.life -= hdt;
                if (d.kind != 1 && d.v.y < 0) {
                    const int bx = int(std::floor(d.p.x * 16.0f)), bz = int(std::floor(d.p.z * 16.0f));
                    float sv = topAt(bx, bz);
                    if (sv <= NONE) {
                        const int fx = (bx >> 4) - flowAt.x, fz = (bz >> 4) - flowAt.y;
                        if (fx >= 0 && fz >= 0 && fx < FLOW_SIZE && fz < FLOW_SIZE) sv = flowSurface[size_t(fz) * FLOW_SIZE + fx] * 16.0f;
                    }
                    if (sv > NONE && d.p.y * 16.0f < sv) {
                        d.life = 0;
                        if (d.kind >= 2 && born.size() < 4000) {   // всплеск: блок воды (или клочок пены) ушёл под гладь
                            const float h = std::max(0.0f, (d.y0 - d.p.y) * 16.0f);   // высота падения, блоки
                            const float vUp = std::min(8.0f, 0.35f * std::sqrt(2.0f * 9.8f * h) + 0.8f);
                            const float k = std::max(1.0f, d.size / 8.0f);   // вдали блоки крупнее - и всплеск
                            const glm::vec2 fv = flowOf(bx, bz);
                            const glm::vec3 at(d.p.x, sv * U, d.p.z);
                            const int n = 1 + int(std::min(h, 6.0f));
                            for (int q = 0; q < n; ++q) {
                                const float a = rnd() * 6.2832f, r = rnd() * (0.6f + vUp * 0.3f);
                                born.push_back(Drop{at, glm::vec3(std::cos(a) * r, vUp * (0.4f + 0.6f * rnd()), std::sin(a) * r) * U,
                                                    0.5f + 0.8f * rnd(), 0, (1.0f + rnd()) * k, at.y});
                            }
                            if (d.kind == 2 && rnd() < 0.2f * std::min(h, 5.0f))
                                born.push_back(Drop{at + glm::vec3(0.0f, 0.3f * U, 0.0f),
                                                    glm::vec3(fv.x * 0.5f + rnd() - 0.5f, 0.5f + rnd(), fv.y * 0.5f + rnd() - 0.5f) * U,
                                                    2.5f + 2.0f * rnd(), 1, (8.0f + 8.0f * rnd()) * k, at.y});
                        }
                    }
                }
            }
            for (const Drop& b : born) if (drops.size() < ALL_CAP) drops.push_back(b);
            // ожидаемое число капель при полной частоте известно наперёд (частота × время жизни по всем кромкам):
            // множитель - чтобы влезть в 85% лимита; меняется плавно, только когда меняются кромки в поле зрения
            dropDemand = demand;
            dropRate += (std::min(1.0f, 0.85f * float(NEAR_CAP) / std::max(1.0f, dropDemand)) - dropRate) * std::min(1.0f, hdt * 2.0f);
            std::erase_if(drops, [](const Drop& d) { return d.life <= 0; });
            };
            // Р75: брызги и пыль живут секунды - картина собиралась на глазах (эффект «копился»). Когда кромки пришли
            // впервые (и на первом кадре), шаги прогоняются вперёд на 3 с без отрисовки - сразу видно установившееся
            if (firstFrame || (lp && !sprayWarmed)) {
                for (int k = 0; k < 30; ++k) sprayStep(0.1f);
                if (lp) sprayWarmed = true;
            }
            sprayStep(std::min(dt, 0.05f));
            dropVerts.clear();
            for (const Chip& c : chips)   // Р93: щепки - кубики в 1/16 блока
                dropVerts.insert(dropVerts.end(), {c.p.x / 16.0f, c.p.y / 16.0f, c.p.z / 16.0f, c.kind == 6 ? 0.005f : 0.004f, 1.0f, float(c.kind), 0.0f, 0.0f});
            for (const Drop& d : drops) {
                const float alpha = d.kind == 0 ? 0.9f : d.kind == 2 ? 0.6f : d.kind == 3 ? 0.85f : 0.28f * std::min(1.0f, d.life);
                dropVerts.insert(dropVerts.end(), {d.p.x, d.p.y, d.p.z, d.size / 256.0f, alpha, float(d.kind), 0.0f, 0.0f});
            }
            gpu->setSpray(dropVerts);
        }
        lap(3);
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
                fineJob = zoneJobs.submit([&tp, want, NONE] {
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
                            // Р61: источник брызг - мокрый блок с перепадом к соседу больше 1/16 блока
                            const float v0 = f->vb[size_t(j + 1) * T + i + 1];
                            if (f->wet[size_t(j + 1) * T + i + 1] && v0 > NONE) {
                                float d = 0;
                                for (const auto [di, dj] : {std::pair{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
                                    const float vn = f->vb[size_t(j + 1 + dj) * T + i + 1 + di];
                                    if (vn > NONE) d = std::max(d, std::abs(vn - v0));
                                }
                                if (d > 1.0f / 16.0f) { f->sources.push_back(want.x + i); f->sources.push_back(want.y + j); }
                            }
                        }
                    return f;
                });
            }
            if (fineJob.valid() && fineJob.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                fineData = take(fineJob, "мелкая вода");
                const FineData& f = *fineData;
                constexpr int T = FINE + 2;
                std::vector<float> tex(size_t(T) * T * 2);
                for (size_t k = 0; k < size_t(T) * T; ++k) { tex[k * 2] = f.vb[k]; tex[k * 2 + 1] = f.ground[k]; }
                std::vector<uint8_t> mask(FINE * FINE, 0);
                std::vector<int32_t> blocks;
                const std::vector<float>& vel = shownVel.empty() ? fieldVel : shownVel;
                // Р59: мелкой водой - не дальше 48 блоков от камеры и не больше 2500 ближайших блоков (иначе на горном
                // потоке ~10 тыс. блоков - 75 млн вершин, кадр 34 мс); дальше тексели и не различить
                std::vector<std::pair<int, int>> cand;   // (квадрат расстояния, индекс)
                for (int j = 0; j < FINE; ++j)
                    for (int i = 0; i < FINE; ++i) {
                        if (!f.wet[size_t(j + 1) * T + i + 1]) continue;
                        const int bx = f.origin.x + i, bz = f.origin.y + j;
                        const int dx = bx - eb.x, dz = bz - eb.y, d2 = dx * dx + dz * dz;
                        if (d2 > 48 * 48) continue;
                        float sp = 0;
                        const int fx = (bx >> 4) - flowAt.x, fz = (bz >> 4) - flowAt.y;
                        if (!vel.empty() && fx >= 0 && fz >= 0 && fx < FLOW_SIZE && fz < FLOW_SIZE)
                            sp = std::hypot(vel[(size_t(fz) * FLOW_SIZE + fx) * 2], vel[(size_t(fz) * FLOW_SIZE + fx) * 2 + 1]);
                        if (f.flat[j * FINE + i] < 1.0f / 32.0f && sp < 1.5f) continue;
                        cand.push_back({d2, j * FINE + i});
                    }
                if (cand.size() > 2500) {
                    std::nth_element(cand.begin(), cand.begin() + 2500, cand.end());
                    cand.resize(2500);
                }
                for (const auto& [d2, k] : cand) {
                    mask[size_t(k)] = 1;
                    blocks.push_back(f.origin.x + k % FINE);
                    blocks.push_back(f.origin.y + k / FINE);
                }
                gpu->setFineWater(f.origin, FINE, tex, mask, blocks);
                fineShown = int(blocks.size() / 2);
                fineData->blocks = blocks;
                simArea = glm::vec4(float(f.origin.x) / 16.0f, float(f.origin.y) / 16.0f, float(FINE) / 16.0f, 1.0f);
                journal::info("вода", "мелкая вода: зона ({}, {}), блоков {}", f.origin.x, f.origin.y, blocks.size() / 2);
            }
        }
        lap(4);
        // Р62: лес. Места - ячейки по 10 блоков: хеш ячейки даёт кандидата в произвольном мелком вокселе (не центр
        // блока); густота - по дождям (климат) и крупному шуму рощ (ячейка 512 блоков: рощи, перелески, поляны); не
        // на воде и не у воды (2 блока), не на крутом (перепад больше 3 блоков на 2), не в снегу. Дуб - по семени
        // ячейки: одна точка мира - всегда тот же дуб.
        {
            const glm::ivec2 eb(int(std::floor(eye.x * 16.0f)), int(std::floor(eye.z * 16.0f)));
            const auto fd = [](int v) { return v >= 0 ? v / 32 : -((-v + 31) / 32); };
            const glm::ivec2 want((fd(eb.x) - 7) * 32, (fd(eb.y) - 7) * 32);
            if (!forestJob.valid() && want != forestAt) {
                forestAt = want;
                forestJob = zoneJobs.submit([&tp, want] {
                    auto z = std::make_unique<ForestZone>();
                    z->origin = want - glm::ivec2(32);   // кайма в кусок - для корней и проверок у края
                    z->side = 17 * 32;
                    const int S = z->side;
                    auto g = std::make_shared<std::vector<int32_t>>(size_t(S) * S);
                    std::vector<int32_t> w(size_t(S) * S);
                    std::vector<world::ColumnHeights> hms(17 * 17);   // Р74: нужны и пещерам
                    for (int cz = 0; cz < 17; ++cz)
                        for (int cx = 0; cx < 17; ++cx) {
                            world::ColumnHeights& hm = hms[size_t(cz) * 17 + cx];
                            world::columnHeights(tp, z->origin.x / 32 + cx, z->origin.y / 32 + cz, hm);
                            // Р79: земля у выхода шахты срезана, поверхность там ниже карты высот. Без этого
                            // дубы вставали по нетронутой высоте и висели в воздухе над сглаженной воронкой.
                            uint8_t cut[world::CHUNK + 2][world::CHUNK + 2];
                            world::soilCut(tp, z->origin.x / 32 + cx, z->origin.y / 32 + cz, hm, cut);
                            for (int j = 0; j < 32; ++j)
                                for (int i = 0; i < 32; ++i) {
                                    (*g)[size_t(cz * 32 + j) * S + cx * 32 + i] = hm.h[j + 1][i + 1] - cut[j + 1][i + 1];
                                    w[size_t(cz * 32 + j) * S + cx * 32 + i] = hm.water[j + 1][i + 1];
                                }
                        }
                    z->ground = g;
                    const auto H = [&](int x, int y) { return (*g)[size_t(y - z->origin.y) * S + (x - z->origin.x)]; };
                    const auto wet = [&](int x, int y) { const size_t k = size_t(y - z->origin.y) * S + (x - z->origin.x); return w[k] > (*g)[k]; };
                    // Р74: воздух пещеры в вокселе - настоящие воксели (caveAir куска), куски - по разу
                    struct CaveChunk { bool any; uint64_t air[world::CHUNK + 2][world::CHUNK + 2]; };
                    std::unordered_map<uint64_t, std::unique_ptr<CaveChunk>> caves;
                    const auto fd32 = [](int v) { return v >= 0 ? v / 32 : -((-v + 31) / 32); };
                    const auto caveAir = [&](int x, int y, int zz) {
                        const int cx = fd32(x), cy = fd32(y), cz = fd32(zz);
                        const uint64_t key = uint64_t(uint32_t(cx)) << 42 ^ uint64_t(uint32_t(cy) & 0x3FFFFF) << 21 ^ uint64_t(uint32_t(cz) & 0x1FFFFF);
                        auto it = caves.find(key);
                        if (it == caves.end()) {
                            auto c = std::make_unique<CaveChunk>();
                            const int lx = cx - z->origin.x / 32, lz = cz - z->origin.y / 32;
                            c->any = lx >= 0 && lz >= 0 && lx < 17 && lz < 17 &&
                                     world::caveAir(tp, world::ChunkPos{cx, cy, cz}, hms[size_t(lz) * 17 + lx], c->air);
                            it = caves.emplace(key, std::move(c)).first;
                        }
                        return it->second->any && ((it->second->air[zz - cz * 32 + 1][x - cx * 32 + 1] >> (y - (cy * 32 - 1))) & 1);
                    };
                    // Р65: места, семена и густота - общие с дальними спрайтами и шумом крон (world/forest)
                    constexpr int CELL = world::FOREST_CELL;
                    const auto fdc = [](int v) { return v >= 0 ? v / CELL : -((-v + CELL - 1) / CELL); };
                    for (int cj = fdc(want.y); cj <= fdc(want.y + 15 * 32 - 1); ++cj)
                        for (int ci = fdc(want.x); ci <= fdc(want.x + 15 * 32 - 1); ++ci) {
                            const world::TreeCandidate tc = world::forestCandidate(tp, ci, cj);
                            const int x = tc.x, y = tc.z;
                            if (x < want.x || y < want.y || x >= want.x + 15 * 32 || y >= want.y + 15 * 32) continue;
                            // вода и берег, крутизна
                            bool bad = false;
                            int32_t lo = INT32_MAX, hi = INT32_MIN;
                            for (int dy = -3; dy <= 3 && !bad; ++dy)   // Р74: вода - не ближе 3 блоков
                                for (int dx = -3; dx <= 3; ++dx) {
                                    if (wet(x + dx, y + dy)) { bad = true; break; }
                                    if (std::abs(dx) > 2 || std::abs(dy) > 2) continue;
                                    lo = std::min(lo, H(x + dx, y + dy));
                                    hi = std::max(hi, H(x + dx, y + dy));
                                }
                            if (bad) { ++z->nWet; continue; }
                            if (hi - lo > 3) { ++z->nSteep; continue; }
                            // Р79: на осыпавшейся земле у выхода шахты дубов нет - там голая порода.
                            // То же правило у дальних спрайтов (world/forest.cpp), иначе на границе
                            // уровней детализации лес то появлялся бы, то пропадал.
                            if (world::soilGone(tp, x, y, H(x, y))) { ++z->nCave; continue; }
                            const float dens = world::forestDensity(tp, x, y, H(x, y));
                            if (dens <= 0.0f) { ++z->nSnow; continue; }
                            ++z->nC;
                            if (tc.roll >= dens) { ++z->nSparse; continue; }
                            // Р74: не над шахтой - под стволом (2×2, 4 блока вглубь) и вокруг него (5×5, верхние 2) нет
                            // воздуха пещеры. Раньше - шум устья в одной точке (Р70): пропускал часть настоящих ходов
                            bool cave = false;
                            for (int dy = -2; dy <= 2 && !cave; ++dy)
                                for (int dx = -2; dx <= 2 && !cave; ++dx) {
                                    const bool under = dx >= 0 && dx <= 1 && dy >= 0 && dy <= 1;
                                    const int top = H(x + dx, y + dy);
                                    for (int d = 0; d < (under ? 4 : 2) && !cave; ++d) cave = caveAir(x + dx, top - d, y + dy);
                                }
                            if (cave) { ++z->nCave; continue; }
                            int32_t g0 = INT32_MAX;
                            for (const auto [dx, dy] : {std::pair{0, 0}, {1, 0}, {0, 1}, {1, 1}}) g0 = std::min(g0, H(x + dx, y + dy));
                            z->spots.push_back({(uint64_t(uint32_t(ci)) << 32) | uint32_t(cj), glm::ivec3(x, g0 + 1, y), glm::ivec3(tc.fx, 0, tc.fz), tc.seed, tc.age});
                        }
                    // Р85: КАРТА ТЕНЕВЫХ ВЫСОТ. Сперва верхи: на каждый столбец высота того, что
                    // закрывает небо. Дерева в блочной маске нет, оно живёт отдельной сеткой,
                    // поэтому крону рисуем сюда кружком: иначе тени под деревьями не будет вовсе, а
                    // она и нужна больше всего.
                    std::vector<float> top(size_t(S) * S);
                    for (size_t i = 0; i < top.size(); ++i) top[i] = float((*g)[i]);
                    for (const ForestSpot& sp : z->spots) {
                        const int cxp = sp.base.x - z->origin.x, czp = sp.base.z - z->origin.y;
                        // Р91: крона по возрасту - та же мера, что у спрайта
                        float shh, sww;
                        world::treeSpriteSize(sp.age, float(sp.seed & 0xFFFF) / 65535.0f, shh, sww);
                        const int canopy = sp.base.y + int(shh * 0.8f), r = int(sww * 0.42f);
                        for (int dz = -r; dz <= r; ++dz)
                            for (int dx = -r; dx <= r; ++dx) {
                                if (dx * dx + dz * dz > r * r) continue;
                                const int px = cxp + dx, pz = czp + dz;
                                if (px < 0 || pz < 0 || px >= S || pz >= S) continue;
                                float& v = top[size_t(pz) * S + px];
                                v = std::max(v, float(canopy));
                            }
                    }
                    // Затем сама тень: идём от столбца к солнцу по верхам и запоминаем, до какой
                    // высоты солнце не достаёт. Считать это ЗАРАНЕЕ, а не лучом в обработчике
                    // фрагментов, - единственный способ получить ровную тень: у луча шаг крупный,
                    // набор отсчётов у каждой точки экрана свой, и тени рвались и мерцали от любого
                    // смещения камеры.
                    const glm::vec3 sun = glm::normalize(glm::vec3(0.4f, 1.0f, 0.25f));
                    const glm::vec2 d(sun.x / sun.y, sun.z / sun.y);   // блоков вбок на блок вверх
                    z->skyTop.assign(size_t(S) * S, 0.0f);
                    constexpr int REACH = 40;   // насколько высоко прослеживаем помеху (блоки)
                    for (int j = 0; j < S; ++j)
                        for (int i = 0; i < S; ++i) {
                            float sh = top[size_t(j) * S + i];
                            for (int k = 1; k <= REACH; ++k) {
                                const int px = i + int(d.x * float(k) + 0.5f), pz = j + int(d.y * float(k) + 0.5f);
                                if (px < 0 || pz < 0 || px >= S || pz >= S) break;
                                sh = std::max(sh, top[size_t(pz) * S + px] - float(k));
                            }
                            z->skyTop[size_t(j) * S + i] = sh;
                        }
                    return z;
                });
            }
            if (firstFrame && forestJob.valid()) forestJob.wait();
            if (forestJob.valid() && forestJob.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                const std::unique_ptr<ForestZone> z = take(forestJob, "зона леса");
                forestGround = z->ground;
                forestGroundOrigin = z->origin;
                forestGroundSide = z->side;
                gpu->setSkyTop(z->side, z->skyTop);   // Р85: по ней фрагментный обработчик строит тени
                skyArea = glm::vec4(float(z->origin.x), float(z->origin.y), float(z->side), 0.0f);
                std::unordered_map<uint64_t, bool> wanted;
                for (const ForestSpot& sp : z->spots) wanted[sp.key] = true;
                for (auto it = forest.begin(); it != forest.end();) {   // ушедшие из зоны - выгрузить
                    if (wanted.count(it->first)) { ++it; continue; }
                    for (const OakMesh& m : it->second.meshes) gpu->destroyMesh(m.mesh);
                    it = forest.erase(it);
                }
                for (const ForestSpot& sp : z->spots)
                    if (!forest.count(sp.key)) forest[sp.key].spot = sp;
                spritesDirty = true;
                journal::info("лес", "зона ({}, {}): мест {}; отказ: вода {}, круто {}, пещера {}, снег и сушь {}, редко {} (из {})",
                              want.x, want.y, z->spots.size(), z->nWet, z->nSteep, z->nCave, z->nSnow, z->nSparse, z->nC);
            }
            // Р63: уровень дерева - по расстоянию: до 12 блоков - 0 (мелкий воксель), 32 - 1, 64 - 2, 128 - 3, дальше - 4
            // (воксель - блок). Сначала - деревья без сетки (грубым уровнем, сразу видно весь лес), затем ближние.
            if (forestGround) {
                // Р70: расстояние - в объёме (с высотой камеры: сверху деревья под тобой грубые); вне поля зрения
                // дерево не уточняется дальше уровня 2; на скорости (> 30 блоков/с - 2, > 120 - 3) тоже
                const float camSpeed = dt > 0 ? glm::length(eyeF() - forestEyePrev) * 16.0f / dt : 0.0f;
                forestEyePrev = eye;
                const int speedMin = camSpeed > 120.0f ? 3 : camSpeed > 30.0f ? 2 : 0;
                const float ey = eye.y * 16.0f;
                const auto levelOf = [&](const ForestTree& t) {
                    const float dx = float(t.spot.base.x - eb.x), dz = float(t.spot.base.z - eb.y), dy = float(t.spot.base.y) + 5.0f - ey;
                    const float d2 = dx * dx + dy * dy + dz * dz;
                    int l = d2 < 12 * 12 ? 0 : d2 < 32 * 32 ? 1 : d2 < 64 * 64 ? 2 : d2 < 128 * 128 ? 3 : 4;
                    const glm::vec3 c(float(t.spot.base.x) / 16.0f, (float(t.spot.base.y) + 5.0f) / 16.0f, float(t.spot.base.z) / 16.0f);
                    if (!fr.visible(c - glm::vec3(0.45f), c + glm::vec3(0.45f))) l = std::max(l, 2);
                    return std::max(l, speedMin);
                };
                std::vector<std::pair<int, ForestTree*>> need;   // (приоритет, дерево)
                for (auto& [key, t] : forest) {
                    if (t.building >= 0) continue;
                    const int want = levelOf(t);
                    if (t.level == want && t.leaves == leavesOn && !t.dirty) continue;
                    const int dx = t.spot.base.x - eb.x, dz = t.spot.base.z - eb.y, d2 = dx * dx + dz * dz;
                    need.push_back({(t.level < 0 ? 0 : 1 << 30) + d2, &t});   // без сетки - первыми
                }
                std::sort(need.begin(), need.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
                const auto gr = forestGround;
                const glm::ivec2 go = forestGroundOrigin;
                const int gs = forestGroundSide;
                for (const auto& [pri, tp2] : need) {
                    if (forestRunning.size() >= 8) break;
                    ForestTree& t = *tp2;
                    // дерево без сетки - сначала грубым уровнем (миллисекунды), уточнится следующим заходом
                    const int lvl = t.level < 0 ? 4 : levelOf(t);
                    t.building = lvl;
                    const ForestSpot sp = t.spot;
                    const auto skel = t.skel;
                    const bool lv = leavesOn;
                    const auto ci = treeCuts.find(sp.key);
                    const game::TreeCut cut = ci != treeCuts.end() ? ci->second : game::TreeCut{};
                    forestRunning.push_back(treeJobs.submit([sp, lvl, gr, go, gs, skel, lv, cut] {
                        ForestBuilt fb{sp.key, lvl, sp.base, {}, skel ? skel : world::growTree(sp.seed, sp.age), lv};
                        // Р93: подрубленное - с зарубкой; срубленное - только пень (ниже реза, с корнями)
                        buildTreeChunks(*fb.skel, sp.base, sp.shift, [&](int32_t bx, int32_t bz) {
                            const int i = bx - go.x, j = bz - go.y;
                            return i < 0 || j < 0 || i >= gs || j >= gs ? INT32_MIN : (*gr)[size_t(j) * gs + i];
                        }, fb.chunks, cut.notched || cut.scraped ? 1 : 1 << lvl, lv, [&cut, &fb](float x, float y, float z, int b) -> int {
                            if (cut.inNotch(x, y, z) || cut.dug) return 0;
                            // Р102: содранная с живого дерева кора - на высоте 0.5..2.5 м сектор ствола, растущий с каждым ударом
                            if (b == 0 && cut.scraped > 0 && y >= 8 && y <= 40) {
                                glm::vec2 c;
                                float R;
                                game::trunkAt(*fb.skel, 24.0f, c, R);
                                const int maxHits = fb.skel->age == world::TreeAge::YOUNG ? 12 : 2 * std::max(1, int(2.0f * R / 4.0f + 0.5f)) * 2;
                                const float frac = std::min(1.0f, float(cut.scraped) / float(maxHits));
                                const float a = std::atan2(z - c.y, x - c.x) / 6.2831853f + 0.5f;
                                if (a <= frac) return 2;
                            }
                            // Р96: перерубленные корни ушли корягами
                            if (b > 0 && b < int(cut.rootOf.size()) && cut.rootOf[size_t(b)] >= 0 && (cut.rootsCut >> cut.rootOf[size_t(b)] & 1u)) return 0;
                            return !cut.felled || !cut.falls(b, y) ? 1 : 0;   // Р94: у пня - всё, что не упало (по скелету)
                        }, &cut);
                        return fb;
                    }));
                }
            }
            size_t uploadBytes = 0;   // Р70: не больше 4 МБ сеток леса за кадр - без рывков
            for (auto it = forestRunning.begin(); it != forestRunning.end();) {
                if (uploadBytes > (4u << 20)) break;
                if (it->wait_for(std::chrono::seconds(0)) != std::future_status::ready) { ++it; continue; }
                ForestBuilt fb = take(*it, "сетка дуба");
                for (const OakChunk& oc : fb.chunks) uploadBytes += oc.verts.size() * 4;
                it = forestRunning.erase(it);
                auto ft = forest.find(fb.key);
                if (ft == forest.end()) continue;
                ForestTree& t = ft->second;
                for (const OakMesh& m : t.meshes) gpu->destroyMesh(m.mesh);   // старая сетка - только теперь
                t.meshes.clear();
                for (const OakChunk& oc : fb.chunks) t.meshes.push_back({gpu->uploadChunkMesh(oc.verts, 0), fb.base, oc.fineOff, oc.scale});
                t.skel = fb.skel;
                t.leaves = fb.leaves;
                t.dirty = false;
                forestUploads += int(fb.chunks.size());
                if (t.level < 0) spritesDirty = true;   // был спрайтом - теперь сетка
                t.level = fb.level;
                t.building = -1;
            }
            // замер: когда все деревья зоны получили сетку - один раз в журнал (за сколько и по уровням)
            {
                static bool told = false;
                static auto since = clock_t_::now();
                if (forest.empty()) { told = false; since = clock_t_::now(); }
                int ready = 0, byLevel[5] = {};
                for (const auto& [key, t] : forest) if (t.level >= 0) { ++ready; ++byLevel[t.level]; }
                if (!told && !forest.empty() && ready == int(forest.size())) {
                    told = true;
                    journal::info("лес", "весь лес зоны с сеткой: {} дубов за {:.0f} мс; по уровням 0-4: {} {} {} {} {}", ready, msSince(since),
                                  byLevel[0], byLevel[1], byLevel[2], byLevel[3], byLevel[4]);
                }
            }
        }
        lap(5);
        // Р65: спрайты дубов - кольцо от зоны леса вблизи (480 блоков) до 1.6 км: те же ячейки, семена и густота
        // (world/forest), земля - по полю рельефа (клетка 16 блоков, между клетками - плавно), без воды и круч
        if (!flowGround.empty()) {
            const glm::vec2 ebf(eye.x * 16.0f, eye.z * 16.0f);
            if (!spriteJob.valid() && (glm::distance(ebf, spriteAt) > 64.0f || spriteZoneAt != forestAt)) {
                spriteAt = ebf;
                spriteZoneAt = forestAt;
                spriteJob = zoneJobs.submit([&tp, cache = spriteCache, za = forestAt, ebf, vr = voxelBlocks * 16, detail = lod.detail] {
                    const auto t0 = clock_t_::now();
                    // Р68: ближнее кольцо - до 1.6 км, каждая ячейка; дальнее - до 3.6 км, чётные ячейки
                    constexpr int R = 1600, RF = 3600, S = world::SPRITE_REGION;
                    const int ex = int(ebf.x), ez = int(ebf.y);
                    const auto fdr = [](int v) { return v >= 0 ? v / S : -((-v + S - 1) / S); };
                    struct Need { int rx, rz, mode; uint64_t key; };
                    std::vector<Need> need;
                    std::vector<uint64_t> keep;
                    for (int rz = fdr(ez - RF); rz <= fdr(ez + RF); ++rz)
                        for (int rx = fdr(ex - RF); rx <= fdr(ex + RF); ++rx) {
                            const float d = std::hypot((rx + 0.5f) * S - ex, (rz + 0.5f) * S - ez);
                            if (d - 0.71f * S > RF) continue;
                            // область целиком в зоне дубов вблизи - там спрайтов нет
                            if (rx * S >= za.x && rz * S >= za.y && (rx + 1) * S <= za.x + 15 * 32 && (rz + 1) * S <= za.y + 15 * 32) continue;
                            int L = 0;   // уровень плиток на этом расстоянии (Р20: дальность уровня L - detail·2^L прежних блоков)
                            while (double(detail) * 16.0 * double(1 << L) < d && L < 12) ++L;
                            const int mode = world::spriteMode(d < float(vr), d > float(R), L);
                            const uint64_t key = uint64_t(uint32_t(rx)) << 32 | uint32_t(rz);
                            keep.push_back(key);
                            const auto it = cache->find(key);
                            if (it == cache->end() || it->second.mode != mode) need.push_back({rx, rz, mode, key});
                        }
                    // новые области - параллельно (первый раз их сотни, дальше - полоса по краю)
                    std::vector<std::vector<float>> out(need.size());
                    std::atomic<size_t> next{0};
                    const auto work = [&] {
                        for (size_t k; (k = next++) < need.size();) out[k] = world::buildSpriteRegion(tp, need[k].rx, need[k].rz, need[k].mode);
                    };
                    if (need.size() > 4) {
                        std::vector<std::thread> th;
                        // первый раз (сотни областей, кадр ждёт) - все ядра, дальше - половина
                        const unsigned hw = std::max(2u, std::thread::hardware_concurrency());
                        for (unsigned t = 0; t < (need.size() > 64 ? hw : hw / 2); ++t) th.emplace_back(work);
                        for (auto& t : th) t.join();
                    } else {
                        work();
                    }
                    for (size_t k = 0; k < need.size(); ++k) (*cache)[need[k].key] = {need[k].mode, std::move(out[k])};
                    const std::unordered_set<uint64_t> kept(keep.begin(), keep.end());
                    for (auto it = cache->begin(); it != cache->end();) it = kept.count(it->first) ? std::next(it) : cache->erase(it);
                    std::vector<float> inst;
                    for (const uint64_t key : keep) {
                        const std::vector<float>& v = (*cache)[key].inst;
                        for (size_t k = 0; k + 8 <= v.size(); k += 8) {
                            const float x = v[k] * 16.0f, z = v[k + 2] * 16.0f;
                            if (x >= za.x && z >= za.y && x < za.x + 15 * 32 && z < za.y + 15 * 32) continue;   // там - дубы вблизи
                            const float dx = x - ex, dz = z - ez;
                            if (dx * dx + dz * dz > float(RF) * RF) continue;
                            inst.insert(inst.end(), v.begin() + k, v.begin() + k + 8);
                        }
                    }
                    journal::info("лес", "спрайты: {} за {:.0f} мс (областей заново {} из {})", inst.size() / 8, msSince(t0), need.size(), keep.size());
                    return inst;
                });
            }
            if (firstFrame && spriteJob.valid()) spriteJob.wait();
            if (spriteJob.valid() && spriteJob.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                spriteFar = take(spriteJob, "спрайты");
                spritesDirty = true;
            }
            // кольцо + дубы зоны, у которых ещё нет сетки (строятся) - спрайтами: лес виден сразу и целиком
            if (spritesDirty && (firstFrame || msSince(spritesAt) > 250.0)) {
                spritesAt = clock_t_::now();
                spritesDirty = false;
                std::vector<float> inst = spriteFar;
                for (const auto& [key, t] : forest) {
                    if (t.level >= 0) continue;
                    const ForestSpot& sp = t.spot;
                    const float fr = float(sp.seed & 0xFFFF) / 65535.0f;
                    float sh, sw;
                    world::treeSpriteSize(sp.age, fr, sh, sw);
                    inst.insert(inst.end(), {(float(sp.base.x) + (sp.shift.x + 0.5f) / 16.0f) / 16.0f, (float(sp.base.y) - 0.2f) / 16.0f,
                                             (float(sp.base.z) + (sp.shift.z + 0.5f) / 16.0f) / 16.0f, sh / 16.0f,
                                             sw / 16.0f, float(sp.seed & 0x7FFF), 0.0f, 0.0f});
                }
                gpu->setTreeSprites(inst);
                spriteCount = inst.size() / 8;
            }
        }
        lap(6);
        frame.reset();
        frame.clear(sky);
        frame.camera({viewProjGpu, glm::vec3(0.4f, 1.0f, 0.25f), eye, voxels.window(),
                      float(lod.drawDistance) * 0.85f, float(lod.drawDistance), 9000.0f, 0.6f, sky,
                      std::chrono::duration<float>(now - startAt).count(), flowArea, simArea, skyArea, originV});
        chunksDrawn = voxels.draw(fr, frame);
        for (const OakMesh& m : oakMeshes) frame.drawFineChunk(m.mesh, m.base, m.fineOff);
        // Р92: предметы - сетка вокруг центра масс, поворот и место - в вызове
        const auto drawItem = [&](glm::dvec3 pos, glm::quat rq, const std::vector<render::MeshId>& ms, const std::vector<glm::ivec3>& offs,
                                  int res) {
            const glm::ivec3 base(int(std::floor(pos.x)), int(std::floor(pos.y)), int(std::floor(pos.z)));
            const glm::mat3 R = glm::mat3_cast(rq) * (16.0f / float(res));   // воксель модели - 1/res блока
            const glm::vec3 dst((pos - glm::dvec3(originV)) / 16.0);
            const glm::vec4 rows[3] = {glm::vec4(R[0][0], R[1][0], R[2][0], dst.x), glm::vec4(R[0][1], R[1][1], R[2][1], dst.y),
                                       glm::vec4(R[0][2], R[1][2], R[2][2], dst.z)};
            for (size_t i = 0; i < ms.size(); ++i) frame.drawFineChunkXf(ms[i], base, offs[i], rows);
        };
        for (game::WorldItem& w : items.list) {
            if (w.meshes.empty()) {
                std::vector<game::ItemMeshChunk> ch;
                game::meshItem(*w.item.model, ch);
                for (const game::ItemMeshChunk& c : ch) { w.meshes.push_back(gpu->uploadChunkMesh(c.verts, 0)); w.offs.push_back(c.off); }
            }
            drawItem(w.body.pos, w.body.rot, w.meshes, w.offs, w.item.model->res);
        }
        // Р94: поваленные деревья - поза тела (центр масс в мире и поворот); сетка построена от блока-основы
        for (const FallenTree& ft : fallen) {
            const game::TreeBody& tb = ft.tb;
            const glm::mat3 R = tb.R();
            const glm::vec3 cq = (tb.com + glm::vec3(tb.shift.x, -3, tb.shift.z)) / 256.0f;
            const glm::vec3 dst = glm::vec3((tb.body.pos - glm::dvec3(originV)) / 16.0) - R * cq;
            const glm::vec4 rows[3] = {glm::vec4(R[0][0], R[1][0], R[2][0], dst.x), glm::vec4(R[0][1], R[1][1], R[2][1], dst.y),
                                       glm::vec4(R[0][2], R[1][2], R[2][2], dst.z)};
            for (size_t i = 0; i < ft.meshes.size(); ++i) frame.drawFineChunkXf(ft.meshes[i], tb.base, ft.offs[i], rows, ft.scales[i]);
            // контур вырубки сука под прицелом
            if (ft.contour >= 0 && ft.cModel)
                drawItem(tb.toWorld(ft.cCorner + glm::vec3(ft.cModel->center) + 0.5f), tb.body.rot, ft.cMeshes, ft.cOffs, 16);
        }
        // Р95: ПРИЦЕЛ - крестик посреди экрана: крошечная модель (воксель 1/512 блока) в 0.4 блока перед глазами
        {
            static std::shared_ptr<game::ItemModel> cross;
            static std::vector<render::MeshId> cm;
            static std::vector<glm::ivec3> co;
            if (!cross) {
                cross = std::make_shared<game::ItemModel>();
                cross->res = 512;
                cross->sx = 9; cross->sy = 9; cross->sz = 1;
                cross->v.assign(81, world::AIR);
                for (int k = 0; k < 9; ++k) if (k < 3 || k > 5) { cross->at(k, 4, 0) = world::HIGHLIGHT; cross->at(4, k, 0) = world::HIGHLIGHT; }
                cross->at(4, 4, 0) = world::HIGHLIGHT;
                cross->finish();
                std::vector<game::ItemMeshChunk> ch;
                game::meshItem(*cross, ch);
                for (const game::ItemMeshChunk& c : ch) { cm.push_back(gpu->uploadChunkMesh(c.verts, 0)); co.push_back(c.off); }
            }
            const glm::vec3 up = glm::cross(right, fwdV);
            const glm::mat3 cam(glm::normalize(glm::cross(fwdV, up)), up, -fwdV);
            drawItem(eye * 16.0 + glm::dvec3(fwdV * 0.4f), glm::quat_cast(cam), cm, co, 512);
        }
        if (craft.stage == 1) {   // Р100: рамка едет за прицелом - на верх блока под ним
            const glm::dvec3 st = eye * 16.0;
            const glm::dvec3 dr(fwd);
            for (int i = 1; i <= 5 * 16; ++i) {
                const glm::dvec3 v = st + dr * (double(i) / 16.0);
                const int32_t bx = int32_t(std::floor(v.x)), by = int32_t(std::floor(v.y)), bz = int32_t(std::floor(v.z));
                if (!voxelQuery.solidTerrain(bx, by, bz)) continue;
                if (!voxelQuery.solid(bx, by + 1, bz))
                    craft.corner = glm::dvec3(std::floor(v.x * 4.0) / 4.0 - 0.25, by + 1.0, std::floor(v.z * 4.0) / 4.0 - 0.25);
                break;
            }
        }
        // Р99: тёска - камень у глаз; крафт - рамка ячеек на земле
        if (knap) {
            static uint64_t knapSig = ~0ull;
            static std::vector<render::MeshId> km;
            static std::vector<glm::ivec3> ko;
            static std::shared_ptr<game::ItemModel> kmod;
            uint64_t sig = uint64_t(knap->cx + 1) | uint64_t(knap->cy + 1) << 3 | uint64_t(knap->rock) << 6;
            for (int y = 0; y < 5; ++y) for (int x = 0; x < 5; ++x) sig = sig * 3 + (knap->cell[y][x] ? 1 : 2);
            if (sig != knapSig) {
                knapSig = sig;
                for (render::MeshId m : km) gpu->destroyMesh(m);
                km.clear(); ko.clear();
                std::vector<game::ItemMeshChunk> ch;
                kmod = game::knapModel(*knap);
                game::meshItem(*kmod, ch);
                for (const game::ItemMeshChunk& c : ch) { km.push_back(gpu->uploadChunkMesh(c.verts, 0)); ko.push_back(c.off); }
            }
            const glm::vec3 up = glm::cross(right, fwdV);
            const glm::mat3 cam(glm::normalize(glm::cross(fwdV, up)), up, -fwdV);
            // середина плиты (8.5, 8.5, 1.5) - точно перед глазами, как бы ни сместился центр масс от сколов
            const glm::quat kq = glm::quat_cast(cam);
            const glm::vec3 com = glm::vec3(kmod->center) + 0.5f, geo(8.5f, 8.5f, 1.5f);
            drawItem(eye * 16.0 + glm::dvec3(fwdV * 0.34f - up * 0.03f + glm::mat3_cast(kq) * ((com - geo) / 64.0f)), kq, km, ko, 64);
        }
        if (craft.stage > 0) {
            static std::vector<render::MeshId> cm;
            static std::vector<glm::ivec3> co;
            if (cm.empty()) {
                auto g = std::make_shared<game::ItemModel>();
                g->res = 64; g->sx = 33; g->sy = 1; g->sz = 33;
                g->v.assign(size_t(33) * 33, world::AIR);
                for (int a = 0; a < 33; ++a)
                    for (int b = 0; b <= 32; b += 16) { g->at(a, 0, b) = world::HIGHLIGHT; g->at(b, 0, a) = world::HIGHLIGHT; }
                g->finish();
                std::vector<game::ItemMeshChunk> ch;
                game::meshItem(*g, ch);
                for (const game::ItemMeshChunk& c : ch) { cm.push_back(gpu->uploadChunkMesh(c.verts, 0)); co.push_back(c.off); }
            }
            drawItem(craft.corner + glm::dvec3(0.25, 0.01, 0.25), glm::quat(1, 0, 0, 0), cm, co, 64);
            // Р100: сложилось - пятая ячейка с готовым предметом справа
            game::Item out;
            std::string name;
            std::vector<size_t> idx;
            static std::vector<render::MeshId> rm;
            static std::vector<glm::ivec3> ro;
            static const game::ItemModel* rmod = nullptr;
            static std::shared_ptr<const game::ItemModel> rhold;
            if (craft.stage == 2 && craftMatch(out, name, idx)) {
                if (!rhold || rhold->sx != out.model->sx || rhold->v != out.model->v) {
                    for (render::MeshId m : rm) gpu->destroyMesh(m);
                    rm.clear(); ro.clear();
                    rhold = out.model;
                    rmod = rhold.get();
                    std::vector<game::ItemMeshChunk> ch;
                    game::meshItem(*rhold, ch);
                    for (const game::ItemMeshChunk& c : ch) { rm.push_back(gpu->uploadChunkMesh(c.verts, 0)); ro.push_back(c.off); }
                }
                const glm::quat lay = glm::angleAxis(float(handTime * 0.8), glm::vec3(0, 1, 0)) * glm::angleAxis(1.5708f, glm::vec3(1, 0, 0));
                drawItem(craft.corner + glm::dvec3(0.675, 0.18 + 0.03 * std::sin(handTime * 2.0), 0.25), lay, rm, ro, rmod->res);
                drawItem(craft.corner + glm::dvec3(0.675, 0.01, 0.25), glm::quat(1, 0, 0, 0), cm, co, 64 * 2);   // рамка готового - вдвое меньше
            }
        }
        // Р92: в руках - у камеры: правая справа внизу, левая слева; черенок наклонён от себя
        for (int hnd = 0; hnd < 2; ++hnd) {
            HandMesh& hm = handMesh[hnd];
            const game::ItemModel* want = hands[hnd] && !(knap && knap->hand == hnd) ? hands[hnd]->model.get() : nullptr;
            if (hm.model != want) {
                for (render::MeshId m : hm.meshes) gpu->destroyMesh(m);
                hm.meshes.clear();
                hm.offs.clear();
                hm.model = want;
                if (want) {
                    std::vector<game::ItemMeshChunk> ch;
                    game::meshItem(*want, ch);
                    for (const game::ItemMeshChunk& c : ch) { hm.meshes.push_back(gpu->uploadChunkMesh(c.verts, 0)); hm.offs.push_back(c.off); }
                }
            }
            if (!want) continue;
            // Р92: ХВАТ. Рука держит предмет за рукоять (grip), рукоять уходит от игрока вперёд-вверх, лезвие
            // смотрит внутрь. Покачивание: на месте - еле заметное дыхание, на ходу - шаг (по пройденному пути).
            const glm::vec3 up = glm::cross(right, fwd);
            const float sway = std::sin(handTime * 1.3f) * 0.012f;
            const float stepA = std::sin(walkPhase) * walkAmp, stepB = std::fabs(std::cos(walkPhase)) * walkAmp;
            const float side = hnd ? 1.0f : -1.0f;
            // Р93: УДАР С ЗАМАХОМ ИЗ-ЗА ПЛЕЧА. Вращение - вокруг кисти (хвата), ось ПОПЕРЁК топорища и наклонена
            // по диагонали: замах уносит головку назад за плечо и вверх (0..0.35), удар - резкий проход сверху
            // справа вниз налево лезвием вперёд (0.35..0.5, засчитывается в середине), затем возврат.
            float chopA = 0, lift = 0, thrust = 0;
            if (swing[hnd] > 0) {
                const float ph = 1.0f - swing[hnd] / SWING;
                const auto ease = [](float x) { x = std::clamp(x, 0.0f, 1.0f); return x * x * (3 - 2 * x); };
                if (ph < 0.35f) { const float a = ease(ph / 0.35f); chopA = 1.45f * a; lift = a; }
                else if (ph < 0.5f) { const float b = (ph - 0.35f) / 0.15f; chopA = 1.45f - 2.4f * b; lift = 1.0f - b; thrust = std::sin(b * 3.1416f); }
                else { const float c = ease((ph - 0.5f) / 0.5f); chopA = -0.95f * (1.0f - c); }
            }
            const glm::vec3 hand = fwd * (0.42f - 0.08f * lift + 0.12f * thrust) + right * (0.27f * side + stepA * 0.05f + 0.07f * lift * side) -
                                   up * (0.30f + stepB * 0.035f + sway - 0.16f * lift);
            const glm::mat3 cam(right, up, -fwd);
            const glm::vec3 chopAxis = glm::normalize(glm::vec3(1.0f, -0.48f * side, 0.0f));   // в осях камеры
            const glm::quat rq = glm::quat_cast(cam) * glm::angleAxis(chopA, chopAxis) *
                                 glm::angleAxis(side * 0.28f + sway * 2.0f + stepA * 0.06f, glm::vec3(0, 0, 1)) *
                                 glm::angleAxis(-0.55f + stepB * 0.05f, glm::vec3(1, 0, 0)) *
                                 glm::angleAxis(side * (1.5708f + 0.45f), glm::vec3(0, 1, 0));   // лезвие - вперёд, от игрока
            const game::ItemModel& im = *want;
            const glm::vec3 gl = (im.grip - (glm::vec3(im.center) + 0.5f)) / float(im.res);   // хват от центра, блоки
            const glm::dvec3 hp = eye * 16.0 + glm::dvec3(hand - glm::mat3_cast(rq) * gl);
            drawItem(hp, rq, hm.meshes, hm.offs, im.res);
        }
        for (const auto& [key, t] : forest)
            for (const OakMesh& m : t.meshes) frame.drawFineChunk(m.mesh, m.base, m.fineOff, m.scale);
        for (const auto& [mesh, origin] : rulerMeshes) frame.drawChunk(mesh, origin);
        const auto tFar = clock_t_::now();
        farTerrain.frame(lod, eye, fr, voxels, frame, farStats);
        farBuiltTotal += farStats.built;   // счёт за весь полёт: farStats.built обнуляется сводкой
        part[1] = std::max(part[1], msSince(tFar));
        lap(7);
        const auto tExe = clock_t_::now();
        gpu->execute(frame, fs);
        part[2] = std::max(part[2], msSince(tExe));
        lap(8);
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
                    journal::info("окно", "плиток построено за полёт {}, подменено родителем {}, в кэше {}; столбцов построено {}",
                                  farBuiltTotal, farStats.substituted, farStats.cached, voxels.stats().built);
                    running = false;
                }
            }
        }
        lap(9);
        gpu->present();
        lap(10);
        firstFrame = false;
        // Р64: долгий кадр (больше 50 мс) - в журнал, не чаще раза в секунду; раз в 10 с - сводка
        {
            const double frameMs = double(dt) * 1000.0;
            statSum += frameMs;
            statWorst = std::max(statWorst, frameMs);
            ++statFrames;
            int trees = 0, lv[5] = {};
            for (const auto& [key, t] : forest) if (t.level >= 0) { ++trees; ++lv[t.level]; }
            const auto counts = [&] {
                return std::format("мелкая вода {} блоков, брызг {}, дубов {} (по уровням 0-4: {} {} {} {} {}), строится {}, выгружено сеток леса {}, спрайтов {}; "
                                   "прямоугольников {}, вызовов {}", fineShown, drops.size(), trees, lv[0], lv[1], lv[2], lv[3], lv[4],
                                   forestRunning.size(), forestUploads, spriteCount, fs.quads, fs.drawCalls);
            };
            const double iterMs = msSince(iterAt);   // Р73: свой проход цикла - его части известны
            // не чаще раза в секунду, кроме совсем долгих (больше 150 мс) - те всегда
            if (std::max(iterMs, frameMs) > 50.0 && (msSince(spikeAt) > 1000.0 || std::max(iterMs, frameMs) > 150.0)) {
                spikeAt = clock_t_::now();
                std::string parts;
                for (int i = 0; i < NSECT; ++i)
                    if (sect[i] >= 1.0) parts += std::format("{}{} {:.0f}", parts.empty() ? "" : ", ", SECT[i], sect[i]);
                journal::warn("кадр", "долгий кадр {:.0f} мс, от прошлого {:.0f} мс ({}): {}", iterMs, frameMs, parts, counts());
                if (sect[1] > 30.0)   // Р75: зона вокселей - худшее по её частям с прошлого сброса
                    journal::warn("кадр", "  зона вокселей, худшее: сдвиг окна {:.0f}, очередь {:.0f}, выгрузка {:.0f}, маска {:.0f} мс",
                                  voxels.stats().worstMs[0], voxels.stats().worstMs[1], voxels.stats().worstMs[2], voxels.stats().worstMs[3]);
            }
            if (msSince(statAt) > 10000.0) {
                journal::info("кадр", "за 10 с: кадров {}, средний {:.1f} мс, худший {:.0f} мс; {}", statFrames, statSum / std::max(1, statFrames), statWorst, counts());
                statAt = clock_t_::now();
                statSum = statWorst = 0;
                statFrames = 0;
            }
            forestUploads = 0;
        }

        if (now - titleAt > std::chrono::milliseconds(500)) {
            const double ms = sumMs / frames;
            // Подсказка разработчика: что под прицелом. Считается тем же кодом, что строит куски,
            // поэтому дорого - только здесь, раз в полсекунды.
            aim = dev::aimedAt(tp, eye, yaw, pitch, 64.0f);
            // Панель отладки: перерисовываем, только если строки изменились - рисование текста системой
            // не бесплатное, а меняется оно раз в полсекунды.
            std::vector<std::string> fresh;
            if (showPanel) {
                fresh.push_back(std::format("камера: {:.0f} {:.0f} {:.0f}   поворот {:.0f}°/{:.0f}°   кадр {:.2f} мс",
                                            eye.x, eye.y, eye.z, yaw * 57.2958f, pitch * 57.2958f, ms));
                fresh.push_back(aim.hit ? std::format("под прицелом: {} — ({} {} {}), {:.1f} бл.", aim.name,
                                                      aim.block.x, aim.block.y, aim.block.z, aim.distance)
                                        : std::string("под прицелом: пусто (64 бл.)"));
                if (aim.hit && !aim.detail.empty()) fresh.push_back("состав: " + aim.detail);
                fresh.push_back(std::format("земля под камерой: {:.0f} бл.", dev::groundAt(tp, eye.x, eye.z)));
                if (knap) {
                    fresh.push_back("ТЁСКА: наведите курсор на клетку камня, ЛКМ - сколоть (можно вести с зажатой), E - отложить. Образцы:");
                    for (const std::string& l : game::knapPatternsText()) fresh.push_back("    " + l);
                }
                if (craft.stage > 0) {
                    game::Item out;
                    std::string name;
                    std::vector<size_t> idx;
                    std::string in;
                    if (craft.stage == 2)
                        for (size_t k : (craftMatch(out, name, idx), idx)) in += (in.empty() ? "" : ", ") + game::itemName(items.list[k].item);
                    fresh.push_back(craft.stage == 1 ? "КРАФТ: рамка едет за прицелом - F2 поставить"
                                                     : "КРАФТ: V+клик - положить в ячейку, ЛКМ/ПКМ - взять; F2 - убрать рамку. Лежит: " + (in.empty() ? std::string("ничего") : in));
                    fresh.push_back(craft.stage == 2 && !name.empty() ? "Готово: " + name + " - возьмите из ячейки справа пустой рукой" : game::recipesHint());
                }
                fresh.push_back("E - тёска камня в руке, F2 - крафт на земле, R+клик - второе действие (камень: волокно с молодого дерева; топор: окорка), двойной щелчок - метнуть");
                fresh.push_back(std::format("руки: левая - {}, правая - {}   (ЛКМ/ПКМ взять; V+ЛКМ/ПКМ положить, Q+ЛКМ/ПКМ бросить; Shift присесть, C лечь, Ctrl бег)",
                                            hands[0] ? game::itemName(*hands[0]) : "пусто", hands[1] ? game::itemName(*hands[1]) : "пусто"));
                fresh.push_back(std::format("воксели: столбцов {} (ждут {}), кусков {}   плиток {}",
                                            voxels.stats().ready, voxels.stats().pending, chunksDrawn, fs.tiles));
                fresh.push_back(typing ? std::format("перейти: {}_   (Enter - идти, Esc - отмена)", gotoText)
                                       : std::string("T - перейти по координатам, F3 - убрать панель"));
            }
            if (fresh != panelLines) {
                panelLines = fresh;
                if (panelLines.empty()) {
                    panelImage = {};
                    gpu->setOverlay(0, 0, {});
                } else if (dev::renderLines(panelLines, 15, panelImage)) {
                    gpu->setOverlay(panelImage.w, panelImage.h, panelImage.rgba);
                }
            }
            std::string head;
            if (typing) {
                head = std::format("перейти: {}_   (Enter - идти, Esc - отмена; «x z» или «x y z»)   |   ", gotoText);
            } else if (aim.hit) {
                head = std::format("под прицелом: {}{}{} — ({} {} {}), {:.1f} бл.   |   ", aim.name,
                                   aim.detail.empty() ? "" : " · ", aim.detail,
                                   aim.block.x, aim.block.y, aim.block.z, aim.distance);
            } else {
                head = "под прицелом: пусто (64 бл.)   |   ";
            }
            SDL_SetWindowTitle(window, (head + std::format("Отмель — кадр {:.2f} мс | воксели: столбцов {} (ждут {}), кусков {} ({} прямоуг.) | "
                                                   "плиток {} ({} тр.) | вызовов {}",
                                                   ms, voxels.stats().ready, voxels.stats().pending, chunksDrawn, fs.quads,
                                                   fs.tiles, fs.tileTriangles, fs.drawCalls)).c_str());
            farStats.built = 0;
            farStats.workMs = 0;
            sumMs = 0;
            frames = 0;
            titleAt = now;
        }
    }

    // Р73: сетки - до gpu.reset(): иначе деструкторы плиток и зоны зовут уже удалённый путь отрисовки
    saveNow();   // Р98: на выходе
    farTerrain.release();
    voxels.releaseAll();
    gpu.reset();
    SDL_DestroyWindow(window);
    SDL_Quit();
    journal::info("окно", "закрыто");
    return 0;
}
