// Путь отрисовки OpenGL 4.6 core. Единственное место, где есть типы и вызовы OpenGL (Р8).
#include <glad/gl.h>
#include <SDL3/SDL.h>

#include "journal.h"
#include "render/render.h"

namespace render {
namespace {

// Сообщения драйвера - в наш журнал: ошибки OpenGL видны сразу, а не по чёрному экрану
void GLAD_API_PTR onGlMessage(GLenum, GLenum type, GLuint id, GLenum severity, GLsizei, const GLchar* msg, const void*) {
    if (severity == GL_DEBUG_SEVERITY_NOTIFICATION) return;
    if (type == GL_DEBUG_TYPE_ERROR || severity == GL_DEBUG_SEVERITY_HIGH)
        journal::error("opengl", "#{} {}", id, msg);
    else
        journal::warn("opengl", "#{} {}", id, msg);
}

class GLBackend final : public Backend {
public:
    GLBackend(SDL_Window* w, SDL_GLContext c) : window_(w), context_(c) {}
    ~GLBackend() override { SDL_GL_DestroyContext(context_); }

    void execute(const CommandList& list) override {
        for (const Command& cmd : list.commands) {
            if (auto* c = std::get_if<CmdClear>(&cmd)) {
                glClearColor(c->color.r, c->color.g, c->color.b, c->color.a);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            }
        }
    }

    void present() override { SDL_GL_SwapWindow(window_); }

private:
    SDL_Window* window_;
    SDL_GLContext context_;
};

}  // namespace

std::unique_ptr<Backend> createGL(SDL_Window* window) {
    SDL_GLContext ctx = SDL_GL_CreateContext(window);
    if (!ctx) { journal::error("opengl", "контекст не создан: {}", SDL_GetError()); return nullptr; }
    const int version = gladLoadGL(reinterpret_cast<GLADloadfunc>(SDL_GL_GetProcAddress));
    if (!version) { journal::error("opengl", "функции OpenGL не загрузились"); SDL_GL_DestroyContext(ctx); return nullptr; }
    journal::info("opengl", "OpenGL {}.{}: {} / {}", GLAD_VERSION_MAJOR(version), GLAD_VERSION_MINOR(version),
                  reinterpret_cast<const char*>(glGetString(GL_RENDERER)), reinterpret_cast<const char*>(glGetString(GL_VERSION)));
    glEnable(GL_DEBUG_OUTPUT);
    glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    glDebugMessageCallback(onGlMessage, nullptr);
    SDL_GL_SetSwapInterval(1);   // ждать обновления экрана: не греть видеокарту тысячами пустых кадров
    return std::make_unique<GLBackend>(window, ctx);
}

}  // namespace render
