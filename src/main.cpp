// Отмель: точка входа. Окно, путь отрисовки, цикл кадров со временем кадра в заголовке.
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <chrono>
#include <format>

#include "journal.h"
#include "render/render.h"

int main(int, char**) {
    if (!SDL_Init(SDL_INIT_VIDEO)) { journal::error("окно", "SDL не запустился: {}", SDL_GetError()); return 1; }

    // OpenGL 4.6 core с отладочным выводом драйвера (Р8: 4.6 - ради SPIR-V)
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 6);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    SDL_Window* window = SDL_CreateWindow("Отмель", 1280, 720, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (!window) { journal::error("окно", "окно не создано: {}", SDL_GetError()); return 1; }

    auto gpu = render::createGL(window);
    if (!gpu) return 1;
    journal::info("окно", "запущено");

    render::CommandList frame;
    using clock = std::chrono::steady_clock;
    auto last = clock::now(), titleAt = last;
    double sumMs = 0;
    int frames = 0;
    bool running = true;
    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_EVENT_QUIT) running = false;
            if (ev.type == SDL_EVENT_KEY_DOWN && ev.key.key == SDLK_ESCAPE) running = false;
        }

        frame.reset();
        frame.clear({0.42f, 0.58f, 0.74f, 1.0f});   // цвет неба
        gpu->execute(frame);
        gpu->present();

        // время кадра: среднее за полсекунды - в заголовок окна
        const auto now = clock::now();
        sumMs += std::chrono::duration<double, std::milli>(now - last).count();
        ++frames;
        last = now;
        if (now - titleAt > std::chrono::milliseconds(500)) {
            const double ms = sumMs / frames;
            SDL_SetWindowTitle(window, std::format("Отмель — кадр {:.2f} мс ({:.0f} к/с)", ms, 1000.0 / ms).c_str());
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
