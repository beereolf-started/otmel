// Отмель: точка входа. Окно, мир из зерна (карта высот на Перлине), сетки кусков, летающая камера.
// В заголовке окна - замеры (Р4): время кадра, генерации и сборки сеток, число кусков и прямоугольников.
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <format>
#include <string>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/mat4x4.hpp>

#include "journal.h"
#include "render/render.h"
#include "world/mesher.h"
#include "world/terrain.h"

namespace {

using clock_t_ = std::chrono::steady_clock;
double msSince(clock_t_::time_point t) { return std::chrono::duration<double, std::milli>(clock_t_::now() - t).count(); }

struct ChunkDraw { render::MeshId mesh; glm::ivec3 origin; };

struct WorldStats {
    int columns = 0, chunks = 0, meshed = 0;
    int64_t quads = 0;
    double genMs = 0, meshMs = 0;
};

// Мир R×R столбцов кусков вокруг начала: высоты, заполнение, сетка - и в видеопамять
std::vector<ChunkDraw> buildWorld(render::Backend& gpu, const world::TerrainParams& tp, int radius, WorldStats& st) {
    std::vector<ChunkDraw> draws;
    world::ColumnHeights hm;
    world::Chunk chunk;
    std::vector<uint32_t> verts;
    for (int cz = -radius; cz < radius; ++cz) {
        for (int cx = -radius; cx < radius; ++cx) {
            auto t0 = clock_t_::now();
            world::columnHeights(tp, cx, cz, hm);
            st.genMs += msSince(t0);
            ++st.columns;
            // по высоте - только куски, через которые проходит поверхность (с каймой): ниже - сплошной камень
            // без видимых граней, выше - воздух. Р2: нетронутое не стоит ничего.
            const int cyLo = (hm.minH - 1) >> world::CHUNK_LOG2, cyHi = hm.maxH >> world::CHUNK_LOG2;
            for (int cy = cyLo; cy <= cyHi; ++cy) {
                chunk.pos = {cx, cy, cz};
                auto t1 = clock_t_::now();
                world::fillChunk(hm, chunk);
                st.genMs += msSince(t1);
                auto t2 = clock_t_::now();
                world::MeshStats ms;
                world::buildMesh(chunk, hm, verts, ms);
                st.meshMs += msSince(t2);
                ++st.chunks;
                if (verts.empty()) continue;
                ++st.meshed;
                st.quads += ms.quads;
                draws.push_back({gpu.uploadChunkMesh(verts), {cx * world::CHUNK, cy * world::CHUNK, cz * world::CHUNK}});
            }
        }
    }
    return draws;
}

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
    // --shot: нарисовать несколько кадров, сохранить снимок и выйти (проверка без человека у экрана)
    bool shot = false;
    for (int i = 1; i < argc; ++i) if (std::string(argv[i]) == "--shot") shot = true;
    if (!SDL_Init(SDL_INIT_VIDEO)) { journal::error("окно", "SDL не запустился: {}", SDL_GetError()); return 1; }
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 6);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    int width = 1280, height = 720;
    SDL_Window* window = SDL_CreateWindow("Отмель", width, height, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (!window) { journal::error("окно", "окно не создано: {}", SDL_GetError()); return 1; }

    const std::string shaderDir = std::string(SDL_GetBasePath()) + "shaders";
    auto gpu = render::createGL(window, shaderDir.c_str());
    if (!gpu) return 1;

    // Мир: 64×64 столбца кусков = 128×128 блоков
    world::TerrainParams tp;
    WorldStats ws;
    const auto tw = clock_t_::now();
    auto draws = buildWorld(*gpu, tp, 32, ws);
    journal::info("мир", "столбцов {}, кусков {} (с гранями {}), прямоугольников {}; генерация {:.1f} мс ({:.3f} на кусок), сетки {:.1f} мс ({:.3f} на кусок); всего {:.0f} мс",
                  ws.columns, ws.chunks, ws.meshed, ws.quads, ws.genMs, ws.genMs / std::max(1, ws.chunks), ws.meshMs,
                  ws.meshMs / std::max(1, ws.chunks), msSince(tw));

    // Камера: в блоках. Мышь - взгляд, WASD - движение, пробел/Ctrl - вверх/вниз, Shift - быстрее, Esc - выход
    glm::vec3 eye(0.0f, 60.0f, 0.0f);
    float yaw = -0.6f, pitch = -0.35f;
    // --eye x y z yaw pitch: откуда снимать (для снимков без человека)
    for (int i = 1; i + 5 < argc; ++i)
        if (std::string(argv[i]) == "--eye") {
            eye = {std::stof(argv[i + 1]), std::stof(argv[i + 2]), std::stof(argv[i + 3])};
            yaw = std::stof(argv[i + 4]);
            pitch = std::stof(argv[i + 5]);
        }
    if (!shot) SDL_SetWindowRelativeMouseMode(window, true);
    int frameNo = 0;

    render::CommandList frame;
    render::FrameStats fs;
    auto last = clock_t_::now(), titleAt = last;
    double sumMs = 0;
    int frames = 0;
    bool running = true;
    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_EVENT_QUIT) running = false;
            if (ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_ESCAPE) running = false;
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
        const float speed = (keys[SDL_SCANCODE_LSHIFT] ? 60.0f : 15.0f) * dt;
        if (keys[SDL_SCANCODE_W]) eye += fwd * speed;
        if (keys[SDL_SCANCODE_S]) eye -= fwd * speed;
        if (keys[SDL_SCANCODE_D]) eye += right * speed;
        if (keys[SDL_SCANCODE_A]) eye -= right * speed;
        if (keys[SDL_SCANCODE_SPACE]) eye.y += speed;
        if (keys[SDL_SCANCODE_LCTRL]) eye.y -= speed;

        const glm::mat4 proj = glm::perspective(glm::radians(70.0f), float(width) / float(std::max(1, height)), 0.05f, 1000.0f);
        const glm::mat4 view = glm::lookAt(eye, eye + fwd, glm::vec3(0, 1, 0));

        frame.reset();
        frame.clear({0.55f, 0.72f, 0.88f, 1.0f});
        frame.camera(proj * view, glm::vec3(0.4f, 1.0f, 0.25f));
        for (const ChunkDraw& d : draws) frame.drawChunk(d.mesh, d.origin);
        gpu->execute(frame, fs);
        if (shot && ++frameNo == 5) {
            saveShot(*gpu, width, height, "shot.bmp");
            journal::info("окно", "кадр: вызовов {}, прямоугольников {}", fs.drawCalls, fs.quads);
            running = false;
        }
        gpu->present();

        if (now - titleAt > std::chrono::milliseconds(500)) {
            const double ms = sumMs / frames;
            SDL_SetWindowTitle(window, std::format("Отмель — кадр {:.2f} мс | кусков {} | прямоугольников {} | вызовов {} | генерация {:.0f} мс, сетки {:.0f} мс",
                                                   ms, ws.meshed, fs.quads, fs.drawCalls, ws.genMs, ws.meshMs).c_str());
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
