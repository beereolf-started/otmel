// Путь отрисовки OpenGL 4.6 core. Единственное место, где есть типы и вызовы OpenGL (Р8).
#include <glad/gl.h>
#include <SDL3/SDL.h>

#include <fstream>
#include <iterator>
#include <string>
#include <vector>

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

// Шейдер из SPIR-V (Р8: GLSL собран в SPIR-V при сборке; тот же двоичный код годится и для Vulkan)
GLuint loadSpirv(const std::string& path, GLenum stage) {
    std::ifstream f(path, std::ios::binary);
    std::vector<char> code((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (code.empty()) { journal::error("opengl", "нет шейдера {}", path); return 0; }
    const GLuint s = glCreateShader(stage);
    glShaderBinary(1, &s, GL_SHADER_BINARY_FORMAT_SPIR_V, code.data(), static_cast<GLsizei>(code.size()));
    glSpecializeShader(s, "main", 0, nullptr, nullptr);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[2048] = {};
        glGetShaderInfoLog(s, sizeof(log), nullptr, log);
        journal::error("opengl", "шейдер {}: {}", path, log);
        glDeleteShader(s);
        return 0;
    }
    return s;
}

constexpr int MAX_QUADS = 1 << 17;   // больше прямоугольников в куске 32³ не бывает даже в «шахматке»

struct FrameUbo { float viewProj[16]; float sunDir[4]; };

class GLBackend final : public Backend {
public:
    GLBackend(SDL_Window* w, SDL_GLContext c) : window_(w), context_(c) {}
    ~GLBackend() override {
        for (const Mesh& m : meshes_) if (m.vbo) glDeleteBuffers(1, &m.vbo);
        glDeleteBuffers(1, &ibo_);
        glDeleteBuffers(1, &ubo_);
        glDeleteVertexArrays(1, &vao_);
        glDeleteProgram(program_);
        SDL_GL_DestroyContext(context_);
    }

    bool init(const std::string& shaderDir) {
        const GLuint vs = loadSpirv(shaderDir + "/terrain.vert.spv", GL_VERTEX_SHADER);
        const GLuint fs = loadSpirv(shaderDir + "/terrain.frag.spv", GL_FRAGMENT_SHADER);
        if (!vs || !fs) return false;
        program_ = glCreateProgram();
        glAttachShader(program_, vs);
        glAttachShader(program_, fs);
        glLinkProgram(program_);
        glDeleteShader(vs);
        glDeleteShader(fs);
        GLint ok = 0;
        glGetProgramiv(program_, GL_LINK_STATUS, &ok);
        if (!ok) { journal::error("opengl", "программа рельефа не собралась"); return false; }

        // Вершина - одно целое: атрибут 0, uint, без перевода в float
        glCreateVertexArrays(1, &vao_);
        glEnableVertexArrayAttrib(vao_, 0);
        glVertexArrayAttribIFormat(vao_, 0, 1, GL_UNSIGNED_INT, 0);
        glVertexArrayAttribBinding(vao_, 0, 0);

        // Индексы общие для всех кусков: прямоугольник k - вершины 4k..4k+3, два треугольника
        std::vector<uint32_t> idx(static_cast<size_t>(MAX_QUADS) * 6);
        for (uint32_t q = 0; q < MAX_QUADS; ++q) {
            const uint32_t b = q * 4;
            uint32_t* i = &idx[q * 6];
            i[0] = b; i[1] = b + 1; i[2] = b + 2; i[3] = b; i[4] = b + 2; i[5] = b + 3;
        }
        glCreateBuffers(1, &ibo_);
        glNamedBufferStorage(ibo_, static_cast<GLsizeiptr>(idx.size() * 4), idx.data(), 0);
        glVertexArrayElementBuffer(vao_, ibo_);

        glCreateBuffers(1, &ubo_);
        glNamedBufferStorage(ubo_, sizeof(FrameUbo), nullptr, GL_DYNAMIC_STORAGE_BIT);

        meshes_.push_back({});   // номер 0 - «нет сетки»
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_CULL_FACE);   // задние грани не рисуем: обход вершин - против часовой снаружи
        return true;
    }

    MeshId uploadChunkMesh(std::span<const uint32_t> v) override {
        if (v.empty()) return NO_MESH;
        Mesh m;
        glCreateBuffers(1, &m.vbo);
        glNamedBufferStorage(m.vbo, static_cast<GLsizeiptr>(v.size_bytes()), v.data(), 0);
        m.quads = static_cast<GLsizei>(v.size() / 4);
        meshes_.push_back(m);
        return static_cast<MeshId>(meshes_.size() - 1);
    }

    void execute(const CommandList& list, FrameStats& stats) override {
        stats = {};
        for (const Command& cmd : list.commands) {
            if (auto* c = std::get_if<CmdClear>(&cmd)) {
                glClearColor(c->color.r, c->color.g, c->color.b, c->color.a);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            } else if (auto* cam = std::get_if<CmdCamera>(&cmd)) {
                FrameUbo u{};
                const float* m = &cam->viewProj[0][0];
                for (int i = 0; i < 16; ++i) u.viewProj[i] = m[i];
                u.sunDir[0] = cam->sunDir.x; u.sunDir[1] = cam->sunDir.y; u.sunDir[2] = cam->sunDir.z;
                glNamedBufferSubData(ubo_, 0, sizeof(u), &u);
                glBindBufferBase(GL_UNIFORM_BUFFER, 0, ubo_);
                glUseProgram(program_);
                glBindVertexArray(vao_);
            } else if (auto* d = std::get_if<CmdDrawChunk>(&cmd)) {
                const Mesh& m = meshes_[d->mesh];
                if (!m.vbo) continue;
                glProgramUniform3i(program_, 0, d->originVoxels.x, d->originVoxels.y, d->originVoxels.z);
                glVertexArrayVertexBuffer(vao_, 0, m.vbo, 0, 4);
                glDrawElements(GL_TRIANGLES, m.quads * 6, GL_UNSIGNED_INT, nullptr);
                ++stats.drawCalls;
                stats.quads += m.quads;
            }
        }
    }

    void present() override { SDL_GL_SwapWindow(window_); }
    void resize(int w, int h) override { glViewport(0, 0, w, h); }
    void readPixels(int w, int h, std::vector<uint8_t>& rgb) override {
        rgb.resize(static_cast<size_t>(w) * h * 3);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, rgb.data());
    }

private:
    struct Mesh { GLuint vbo = 0; GLsizei quads = 0; };
    SDL_Window* window_;
    SDL_GLContext context_;
    GLuint program_ = 0, vao_ = 0, ibo_ = 0, ubo_ = 0;
    std::vector<Mesh> meshes_;
};

}  // namespace

std::unique_ptr<Backend> createGL(SDL_Window* window, const char* shaderDir) {
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
    auto backend = std::make_unique<GLBackend>(window, ctx);
    if (!backend->init(shaderDir)) return nullptr;
    return backend;
}

}  // namespace render
