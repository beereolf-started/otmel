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
        if (shot && warm && flySpeed > 0) eye += glm::normalize(glm::vec3(fwd.x, 0, fwd.z)) * flySpeed * dt;

        // обратная глубина (Р20): ближняя и дальняя плоскости переставлены местами
        const float farPlane = float(lod.drawDistance) * 1.5f;
        const glm::mat4 proj = glm::perspectiveRH_ZO(glm::radians(70.0f), float(width) / float(std::max(1, height)), farPlane, 0.05f);
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
        frame.reset();
        frame.clear(sky);
        frame.camera({viewProj, glm::vec3(0.4f, 1.0f, 0.25f), eye, voxels.window(),
                      float(lod.drawDistance) * 0.85f, float(lod.drawDistance), 9000.0f, 0.6f, sky,
                      std::chrono::duration<float>(now - startAt).count(), flowArea, simArea});
        chunksDrawn = voxels.draw(fr, frame);
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
