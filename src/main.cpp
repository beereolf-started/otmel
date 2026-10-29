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
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>

#include "journal.h"
#include "render/render.h"
#include "world/lod.h"
#include "world/mesher.h"
#include "world/terrain.h"

namespace {

using clock_t_ = std::chrono::steady_clock;
double msSince(clock_t_::time_point t) { return std::chrono::duration<double, std::milli>(clock_t_::now() - t).count(); }

struct ChunkDraw { render::MeshId mesh; glm::ivec3 origin; };

struct WorldStats {
    int columns = 0, chunks = 0, meshed = 0, threads = 1;
    int64_t quads = 0;
    double genMs = 0, meshMs = 0;   // суммарное время всех потоков (работа), а не по часам
};

// Мир (2R)×(2R) столбцов кусков вокруг столбца (atX, atZ): высоты, заполнение, сетка - и в видеопамять.
// Столбцы считают все ядра: генерация - чистая функция координат (Р2), порядок счёта на результат не влияет.
// Видеокарта - только из главного потока, поэтому идём полосами строк: посчитали полосу - выгрузили - забыли.
std::vector<ChunkDraw> buildWorld(render::Backend& gpu, const world::TerrainParams& tp, int atX, int atZ, int radius,
                                  WorldStats& st) {
    struct Built { glm::ivec3 origin; std::vector<uint32_t> verts; };
    struct Worker {
        std::vector<Built> done;
        world::ColumnHeights hm;
        world::Chunk chunk;
        WorldStats st;
    };
    const int threads = std::max(1u, std::thread::hardware_concurrency());
    std::vector<std::unique_ptr<Worker>> workers;
    for (int t = 0; t < threads; ++t) workers.push_back(std::make_unique<Worker>());
    std::vector<ChunkDraw> draws;
    constexpr int BAND = 8;   // строк столбцов в полосе
    const int side = 2 * radius;
    for (int band = 0; band < side; band += BAND) {
        std::atomic<int> next{0};
        const int count = std::min(BAND, side - band) * side;
        auto work = [&](Worker& w) {
            std::vector<uint32_t> verts;
            for (int k; (k = next++) < count;) {
                const int cx = atX - radius + k % side, cz = atZ - radius + band + k / side;
                auto t0 = clock_t_::now();
                world::columnHeights(tp, cx, cz, w.hm);
                w.st.genMs += msSince(t0);
                ++w.st.columns;
                // по высоте - только куски, через которые проходит поверхность (с каймой): ниже - сплошной камень
                // без видимых граней, выше - воздух. Р2: нетронутое не стоит ничего.
                const int cyLo = (w.hm.minH - 1) >> world::CHUNK_LOG2, cyHi = w.hm.maxH >> world::CHUNK_LOG2;
                for (int cy = cyLo; cy <= cyHi; ++cy) {
                    w.chunk.pos = {cx, cy, cz};
                    auto t1 = clock_t_::now();
                    world::fillChunk(w.hm, w.chunk);
                    w.st.genMs += msSince(t1);
                    auto t2 = clock_t_::now();
                    world::MeshStats ms;
                    world::buildMesh(w.chunk, w.hm, verts, ms);
                    w.st.meshMs += msSince(t2);
                    ++w.st.chunks;
                    if (verts.empty()) continue;
                    ++w.st.meshed;
                    w.st.quads += ms.quads;
                    w.done.push_back({{cx * world::CHUNK, cy * world::CHUNK, cz * world::CHUNK}, verts});
                }
            }
        };
        std::vector<std::thread> pool;
        for (int t = 0; t < threads; ++t) pool.emplace_back(work, std::ref(*workers[t]));
        for (auto& t : pool) t.join();
        for (auto& w : workers) {
            for (Built& b : w->done) draws.push_back({gpu.uploadChunkMesh(b.verts), b.origin});
            w->done.clear();
        }
    }
    for (auto& w : workers) {
        st.columns += w->st.columns; st.chunks += w->st.chunks; st.meshed += w->st.meshed;
        st.quads += w->st.quads; st.genMs += w->st.genMs; st.meshMs += w->st.meshMs;
    }
    st.threads = threads;
    return draws;
}

// Пирамида видимости из матрицы камеры (Гриббс - Хартманн): шесть плоскостей, точка внутри - если со всеми >= 0.
// Глубина у нас в [0, 1] (обратная, Р20), поэтому плоскости по z - строка 2 и строка 3 минус строка 2.
struct Frustum {
    glm::vec4 planes[6];
    explicit Frustum(const glm::mat4& m) {
        const auto row = [&](int r) { return glm::vec4(m[0][r], m[1][r], m[2][r], m[3][r]); };
        planes[0] = row(3) + row(0);
        planes[1] = row(3) - row(0);
        planes[2] = row(3) + row(1);
        planes[3] = row(3) - row(1);
        planes[4] = row(2);
        planes[5] = row(3) - row(2);
    }
    // коробка видна, если её самый «передний» угол (по нормали плоскости) не позади ни одной плоскости
    bool visible(glm::vec3 lo, glm::vec3 hi) const {
        for (const glm::vec4& p : planes) {
            const glm::vec3 v(p.x >= 0 ? hi.x : lo.x, p.y >= 0 ? hi.y : lo.y, p.z >= 0 ? hi.z : lo.z);
            if (p.x * v.x + p.y * v.y + p.z * v.z + p.w < 0) return false;
        }
        return true;
    }
};

// Дальний рельеф (Р20): плитки CDLOD. Каждый кадр - выбор по расстоянию, отсечение по пирамиде видимости,
// недостающие плитки строятся на всех ядрах (чистая функция координат), лишние вытесняются.
class FarTerrain {
public:
    struct Stats { int built = 0; double buildMs = 0; size_t cached = 0; };

    FarTerrain(render::Backend& gpu, const world::TerrainParams& tp) : gpu_(gpu), tp_(tp) {}
    ~FarTerrain() { for (auto& [k, e] : cache_) gpu_.destroyMesh(e.mesh); }

    void frame(const world::LodSettings& s, glm::vec3 eye, const Frustum& fr, render::CommandList& out, Stats& st) {
        ++frame_;
        world::selectTiles(s, eye.x, eye.z, sel_);
        // отсечение по четвертям; высоту коробки берём из готовой плитки, у неготовой - весь диапазон мира (Р10)
        visible_.clear();
        missing_.clear();
        for (world::TileDraw d : sel_) {
            const int64_t size = world::tileBlocks(d.key.level), half = size / 2;
            const int64_t x0 = int64_t(d.key.tx) * size, z0 = int64_t(d.key.tz) * size;
            auto it = cache_.find(world::tileHash(d.key));
            const bool ready = it != cache_.end();
            const float lo = ready ? it->second.minY : -500.0f, hi = ready ? it->second.maxY : 1000.0f;
            for (int q = 0; q < 4; ++q)
                if (d.quadrants >> q & 1) {
                    const float qx = float(x0 + (q & 1) * half), qz = float(z0 + (q >> 1) * half);
                    if (!fr.visible({qx, lo, qz}, {qx + float(half), hi, qz + float(half)})) d.quadrants &= uint8_t(~(1 << q));
                }
            if (!d.quadrants) continue;
            visible_.push_back(d);
            if (!ready) missing_.push_back(d.key);
        }
        build(st);
        for (const world::TileDraw& d : visible_) {
            Entry& e = cache_[world::tileHash(d.key)];
            e.used = frame_;
            const int L = d.key.level;
            const float end = float(world::lodRange(s, L));
            // морфинг - на последних 30% промежутка между дальностями уровней L-1 и L
            const float start = end - 0.3f * (end - end / 2);
            out.drawTile({e.mesh, {int(int64_t(d.key.tx) * world::tileBlocks(L)), int(int64_t(d.key.tz) * world::tileBlocks(L))},
                          1 << L, L, start, end, d.quadrants});
        }
        evict();
        st.cached = cache_.size();
    }

private:
    struct Entry { render::MeshId mesh = render::NO_MESH; float minY = 0, maxY = 0; uint64_t used = 0; };

    void build(Stats& st) {
        if (missing_.empty()) return;
        const auto t0 = clock_t_::now();
        std::vector<world::TileMesh> meshes(missing_.size());
        std::atomic<size_t> next{0};
        auto work = [&] {
            for (size_t k; (k = next++) < missing_.size();) world::buildTile(tp_, missing_[k], meshes[k]);
        };
        const size_t threads = std::min<size_t>(missing_.size(), std::max(1u, std::thread::hardware_concurrency()));
        std::vector<std::thread> pool;
        for (size_t t = 0; t < threads; ++t) pool.emplace_back(work);
        for (auto& t : pool) t.join();
        for (size_t k = 0; k < missing_.size(); ++k) {
            Entry e;
            e.mesh = gpu_.uploadTile(meshes[k].data);
            e.minY = float(meshes[k].minH) / world::VOXELS_PER_BLOCK;
            e.maxY = float(meshes[k].maxH + 1) / world::VOXELS_PER_BLOCK;
            cache_[world::tileHash(missing_[k])] = e;
        }
        st.built += int(missing_.size());
        st.buildMs += msSince(t0);
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
    const world::TerrainParams& tp_;
    std::unordered_map<uint64_t, Entry> cache_;
    std::vector<world::TileDraw> sel_, visible_;
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
    //   --draw B     дальность видимости, блоков (8192)
    //   --detail K   детальность дальних уровней: дальность уровня 0, блоков (128; не меньше 65 - иначе щели)
    //   --at X Z     точка старта, блоки;  --eye x y z yaw pitch - камера;  --shot - снимок и выход
    bool shot = false;
    int voxelBlocks = 128;
    float atX = 0, atZ = 0;
    world::LodSettings lod;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--shot") shot = true;
        if (a == "--voxels" && i + 1 < argc) voxelBlocks = std::max(2, std::stoi(argv[i + 1]));
        if (a == "--draw" && i + 1 < argc) lod.drawDistance = std::max(256, std::stoi(argv[i + 1]));
        if (a == "--detail" && i + 1 < argc) lod.detail = std::max(65, std::stoi(argv[i + 1]));
        if (a == "--at" && i + 2 < argc) { atX = std::stof(argv[i + 1]); atZ = std::stof(argv[i + 2]); }
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

    // Зона вокселей: (2R)×(2R) столбцов кусков (столбец - 2×2 блока) вокруг точки старта
    const int blocksPerColumn = world::CHUNK / world::VOXELS_PER_BLOCK;
    const int radius = std::max(1, voxelBlocks / blocksPerColumn);
    const int atCX = int(std::floor(atX / blocksPerColumn)), atCZ = int(std::floor(atZ / blocksPerColumn));
    world::TerrainParams tp;
    WorldStats ws;
    const auto tw = clock_t_::now();
    auto draws = buildWorld(*gpu, tp, atCX, atCZ, radius, ws);
    const double wallMs = msSince(tw);
    journal::info("мир", "воксели: столбцов {}, кусков {} (с гранями {}), прямоугольников {}; работа: генерация {:.1f} мс ({:.3f} на кусок), сетки {:.1f} мс ({:.3f} на кусок); по часам {:.0f} мс на {} потоках",
                  ws.columns, ws.chunks, ws.meshed, ws.quads, ws.genMs, ws.genMs / std::max(1, ws.chunks), ws.meshMs,
                  ws.meshMs / std::max(1, ws.chunks), wallMs, ws.threads);
    lod.holeMinX = int64_t(atCX - radius) * blocksPerColumn;
    lod.holeMinZ = int64_t(atCZ - radius) * blocksPerColumn;
    lod.holeMaxX = int64_t(atCX + radius) * blocksPerColumn;
    lod.holeMaxZ = int64_t(atCZ + radius) * blocksPerColumn;
    FarTerrain farTerrain(*gpu, tp);

    // Камера: в блоках. Мышь - взгляд, WASD - движение, пробел/Ctrl - вверх/вниз, Shift - быстрее,
    // колесо - скорость, L - раскраска дальних уровней, Esc - выход. Старт - в 40 блоках над землёй.
    const float ground = float(world::reliefAt(tp, int32_t(atX * 16), int32_t(atZ * 16)).height) / world::VOXELS_PER_BLOCK;
    glm::vec3 eye(atX, ground + 40.0f, atZ);
    float yaw = -0.6f, pitch = -0.35f, speedScale = 1.0f;
    int debug = 0;
    for (int i = 1; i + 5 < argc; ++i)
        if (std::string(argv[i]) == "--eye") {
            eye = {std::stof(argv[i + 1]), std::stof(argv[i + 2]), std::stof(argv[i + 3])};
            yaw = std::stof(argv[i + 4]);
            pitch = std::stof(argv[i + 5]);
        }
    if (!shot) SDL_SetWindowRelativeMouseMode(window, true);
    // в режиме снимка - без ожидания обновления экрана: замер времени кадра не упирается в 60 Гц
    if (shot) SDL_GL_SetSwapInterval(0);
    int frameNo = 0;
    double shotMs = 0;

    const render::Color sky{0.55f, 0.72f, 0.88f, 1.0f};
    render::CommandList frame;
    render::FrameStats fs;
    FarTerrain::Stats farStats;
    auto last = clock_t_::now(), titleAt = last;
    double sumMs = 0;
    int frames = 0, chunksDrawn = 0;
    bool running = true;
    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_EVENT_QUIT) running = false;
            if (ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_ESCAPE) running = false;
            if (ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_L) gpu->setDebug(debug ^= 1);
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
        const float speed = (keys[SDL_SCANCODE_LSHIFT] ? 60.0f : 15.0f) * speedScale * dt;
        if (keys[SDL_SCANCODE_W]) eye += fwd * speed;
        if (keys[SDL_SCANCODE_S]) eye -= fwd * speed;
        if (keys[SDL_SCANCODE_D]) eye += right * speed;
        if (keys[SDL_SCANCODE_A]) eye -= right * speed;
        if (keys[SDL_SCANCODE_SPACE]) eye.y += speed;
        if (keys[SDL_SCANCODE_LCTRL]) eye.y -= speed;

        // обратная глубина (Р20): ближняя и дальняя плоскости переставлены местами
        const float farPlane = float(lod.drawDistance) * 1.5f;
        const glm::mat4 proj = glm::perspectiveRH_ZO(glm::radians(70.0f), float(width) / float(std::max(1, height)), farPlane, 0.05f);
        const glm::mat4 viewProj = proj * glm::lookAt(eye, eye + fwd, glm::vec3(0, 1, 0));
        const Frustum fr(viewProj);

        frame.reset();
        frame.clear(sky);
        frame.camera({viewProj, glm::vec3(0.4f, 1.0f, 0.25f), eye,
                      glm::vec4(float(lod.holeMinX), float(lod.holeMinZ), float(lod.holeMaxX), float(lod.holeMaxZ)),
                      float(lod.drawDistance) * 0.75f, float(lod.drawDistance), sky});
        chunksDrawn = 0;
        for (const ChunkDraw& d : draws) {
            const glm::vec3 lo = glm::vec3(d.origin) / float(world::VOXELS_PER_BLOCK);
            if (!fr.visible(lo, lo + glm::vec3(float(blocksPerColumn)))) continue;
            frame.drawChunk(d.mesh, d.origin);
            ++chunksDrawn;
        }
        farTerrain.frame(lod, eye, fr, frame, farStats);
        gpu->execute(frame, fs);
        // снимок: 5 кадров на разгон, снимок, затем 60 кадров на замер времени кадра
        if (shot) {
            ++frameNo;
            if (frameNo > 5) shotMs += double(dt) * 1000.0;
            if (frameNo == 5) {
                saveShot(*gpu, width, height, "shot.bmp");
                journal::info("окно", "кадр: кусков {}, прямоугольников {}; плиток {}, треугольников {}; вызовов {}; плиток построено {} за {:.0f} мс",
                              chunksDrawn, fs.quads, fs.tiles, fs.tileTriangles, fs.drawCalls, farStats.built, farStats.buildMs);
            }
            if (frameNo == 66) {
                journal::info("окно", "время кадра (60 кадров, без ожидания экрана): {:.2f} мс", shotMs / 60);
                running = false;
            }
        }
        gpu->present();

        if (now - titleAt > std::chrono::milliseconds(500)) {
            const double ms = sumMs / frames;
            SDL_SetWindowTitle(window, std::format("Отмель — кадр {:.2f} мс | кусков {} ({} прямоуг.) | плиток {} ({} тр.) | вызовов {} | "
                                                   "построено плиток {} за {:.0f} мс, в памяти {}",
                                                   ms, chunksDrawn, fs.quads, fs.tiles, fs.tileTriangles, fs.drawCalls,
                                                   farStats.built, farStats.buildMs, farStats.cached).c_str());
            farStats.built = 0;
            farStats.buildMs = 0;
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
