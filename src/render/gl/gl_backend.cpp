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

GLuint loadProgram(const std::string& dir, const char* name) {
    const GLuint vs = loadSpirv(dir + "/" + name + ".vert.spv", GL_VERTEX_SHADER);
    const GLuint fs = loadSpirv(dir + "/" + name + ".frag.spv", GL_FRAGMENT_SHADER);
    if (!vs || !fs) return 0;
    const GLuint p = glCreateProgram();
    glAttachShader(p, vs);
    glAttachShader(p, fs);
    glLinkProgram(p);
    glDeleteShader(vs);
    glDeleteShader(fs);
    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) { journal::error("opengl", "программа {} не собралась", name); glDeleteProgram(p); return 0; }
    return p;
}

constexpr int MAX_QUADS = 1 << 17;   // больше прямоугольников в куске 32³ не бывает даже в «шахматке»
constexpr int TILE = 32, TILE_VERTS = 33;
constexpr int QUAD_INDICES = (TILE / 2) * (TILE / 2) * 6;   // индексов в четверти плитки
constexpr int SAMPLES = 4;                                   // сглаживание MSAA ×4

// std140: всё по 16 байт
struct FrameUbo {
    float viewProj[16];
    float sunDir[4];
    float eye[4];
    float voxelZone[4];
    float fade[4];   // начало, конец растворения вдали
    float sky[4];
};

class GLBackend final : public Backend {
public:
    GLBackend(SDL_Window* w, SDL_GLContext c) : window_(w), context_(c) {}
    ~GLBackend() override {
        for (const Mesh& m : meshes_) if (m.vbo) glDeleteBuffers(1, &m.vbo);
        glDeleteBuffers(1, &ibo_);
        glDeleteBuffers(1, &tileIbo_);
        glDeleteBuffers(1, &ubo_);
        glDeleteVertexArrays(1, &vao_);
        glDeleteVertexArrays(1, &tileVao_);
        glDeleteProgram(program_);
        glDeleteProgram(tileProgram_);
        if (maskTex_) glDeleteTextures(1, &maskTex_);
        destroyTargets();
        SDL_GL_DestroyContext(context_);
    }

    bool init(const std::string& shaderDir, int w, int h) {
        program_ = loadProgram(shaderDir, "terrain");
        tileProgram_ = loadProgram(shaderDir, "tile");
        if (!program_ || !tileProgram_) return false;

        // Вершина куска - одно целое: атрибут 0, uint, без перевода в float
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

        // Узел плитки - три целых (Р20)
        glCreateVertexArrays(1, &tileVao_);
        glEnableVertexArrayAttrib(tileVao_, 0);
        glVertexArrayAttribIFormat(tileVao_, 0, 3, GL_UNSIGNED_INT, 0);
        glVertexArrayAttribBinding(tileVao_, 0, 0);
        // Индексы плитки общие, по четвертям подряд: четверть q - отрезок [q·1536, (q+1)·1536). Диагональ
        // квадрата везде одна, (i, j)-(i+1, j+1): тогда сползшая плитка складывается в грубую с той же диагональю.
        // Обход - против часовой сверху: (i, j) -> (i, j+1) -> (i+1, j+1), как у верхних граней вокселей.
        std::vector<uint16_t> tidx;
        tidx.reserve(4 * QUAD_INDICES);
        for (int q = 0; q < 4; ++q)
            for (int j = (q >> 1) * 16; j < (q >> 1) * 16 + 16; ++j)
                for (int i = (q & 1) * 16; i < (q & 1) * 16 + 16; ++i) {
                    const auto v = [](int a, int b) { return uint16_t(b * TILE_VERTS + a); };
                    const uint16_t a = v(i, j), b = v(i, j + 1), c = v(i + 1, j + 1), d = v(i + 1, j);
                    tidx.insert(tidx.end(), {a, b, c, a, c, d});
                }
        glCreateBuffers(1, &tileIbo_);
        glNamedBufferStorage(tileIbo_, static_cast<GLsizeiptr>(tidx.size() * 2), tidx.data(), 0);
        glVertexArrayElementBuffer(tileVao_, tileIbo_);

        glCreateBuffers(1, &ubo_);
        glNamedBufferStorage(ubo_, sizeof(FrameUbo), nullptr, GL_DYNAMIC_STORAGE_BIT);

        meshes_.push_back({});   // номер 0 - «нет сетки»
        // Обратная глубина (Р20): глубина в [0, 1], ближнее -> 1, дальнее -> 0, в 32-битном числе с плавающей
        // точкой. Точность плавающих чисел гуще у нуля - туда и уходит даль: погрешность почти одинакова
        // на всех расстояниях, а не растёт как квадрат расстояния.
        glClipControl(GL_LOWER_LEFT, GL_ZERO_TO_ONE);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_GREATER);
        glEnable(GL_CULL_FACE);   // задние грани не рисуем: обход вершин - против часовой снаружи
        createTargets(w, h);
        const uint8_t none = 0;
        updateVoxelMask(1, {&none, 1});   // пока вокселей нет - маска 1×1 из нуля
        return true;
    }

    MeshId uploadChunkMesh(std::span<const uint32_t> v) override {
        if (v.empty()) return NO_MESH;
        return store(v, static_cast<GLsizei>(v.size() / 4));
    }

    MeshId uploadTile(std::span<const uint32_t> words) override { return store(words, 0); }

    void destroyMesh(MeshId id) override {
        if (id == NO_MESH || id >= meshes_.size() || !meshes_[id].vbo) return;
        glDeleteBuffers(1, &meshes_[id].vbo);
        meshes_[id] = {};
        free_.push_back(id);
    }

    void setDebug(int mode) override { glProgramUniform1i(tileProgram_, 4, mode); }

    void updateVoxelMask(int side, std::span<const uint8_t> mask) override {
        if (side != maskSide_) {
            if (maskTex_) glDeleteTextures(1, &maskTex_);
            glCreateTextures(GL_TEXTURE_2D, 1, &maskTex_);
            glTextureStorage2D(maskTex_, 1, GL_R8UI, side, side);
            maskSide_ = side;
        }
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTextureSubImage2D(maskTex_, 0, 0, 0, side, side, GL_RED_INTEGER, GL_UNSIGNED_BYTE, mask.data());
    }

    void execute(const CommandList& list, FrameStats& stats) override {
        stats = {};
        GLuint current = 0;
        for (const Command& cmd : list.commands) {
            if (auto* c = std::get_if<CmdClear>(&cmd)) {
                glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
                glViewport(0, 0, width_, height_);
                const float color[4] = {c->color.r, c->color.g, c->color.b, c->color.a};
                const float depth = 0.0f;   // обратная глубина: «бесконечно далеко» - ноль
                glClearNamedFramebufferfv(fbo_, GL_COLOR, 0, color);
                glClearNamedFramebufferfv(fbo_, GL_DEPTH, 0, &depth);
            } else if (auto* cam = std::get_if<CmdCamera>(&cmd)) {
                FrameUbo u{};
                const float* m = &cam->viewProj[0][0];
                for (int i = 0; i < 16; ++i) u.viewProj[i] = m[i];
                u.sunDir[0] = cam->sunDir.x; u.sunDir[1] = cam->sunDir.y; u.sunDir[2] = cam->sunDir.z;
                u.eye[0] = cam->eye.x; u.eye[1] = cam->eye.y; u.eye[2] = cam->eye.z;
                for (int i = 0; i < 4; ++i) u.voxelZone[i] = cam->voxelZone[i];
                u.fade[0] = cam->fadeStart; u.fade[1] = cam->fadeEnd;
                u.sky[0] = cam->sky.r; u.sky[1] = cam->sky.g; u.sky[2] = cam->sky.b; u.sky[3] = cam->sky.a;
                glNamedBufferSubData(ubo_, 0, sizeof(u), &u);
                glBindBufferBase(GL_UNIFORM_BUFFER, 0, ubo_);
            } else if (auto* d = std::get_if<CmdDrawChunk>(&cmd)) {
                const Mesh& m = meshes_[d->mesh];
                if (!m.vbo) continue;
                if (current != program_) { glUseProgram(program_); glBindVertexArray(vao_); current = program_; }
                glProgramUniform3i(program_, 0, d->originVoxels.x, d->originVoxels.y, d->originVoxels.z);
                glVertexArrayVertexBuffer(vao_, 0, m.vbo, 0, 4);
                glDrawElements(GL_TRIANGLES, m.quads * 6, GL_UNSIGNED_INT, nullptr);
                ++stats.drawCalls;
                stats.quads += m.quads;
            } else if (auto* t = std::get_if<CmdDrawTile>(&cmd)) {
                const Mesh& m = meshes_[t->mesh];
                if (!m.vbo) continue;
                if (current != tileProgram_) {
                    glUseProgram(tileProgram_);
                    glBindVertexArray(tileVao_);
                    glBindTextureUnit(0, maskTex_);
                    current = tileProgram_;
                }
                glProgramUniform2i(tileProgram_, 0, t->originBlocks.x, t->originBlocks.y);
                glProgramUniform1i(tileProgram_, 1, t->spacing);
                glProgramUniform2f(tileProgram_, 2, t->morphStart, t->morphEnd);
                glProgramUniform1i(tileProgram_, 3, t->level);
                glVertexArrayVertexBuffer(tileVao_, 0, m.vbo, 0, 12);
                if (t->quadrants == 0xF) {
                    glDrawElements(GL_TRIANGLES, 4 * QUAD_INDICES, GL_UNSIGNED_SHORT, nullptr);
                    ++stats.drawCalls;
                    stats.tileTriangles += 4 * QUAD_INDICES / 3;
                } else {
                    for (int q = 0; q < 4; ++q)
                        if (t->quadrants >> q & 1) {
                            glDrawElements(GL_TRIANGLES, QUAD_INDICES, GL_UNSIGNED_SHORT,
                                           reinterpret_cast<const void*>(static_cast<uintptr_t>(q * QUAD_INDICES * 2)));
                            ++stats.drawCalls;
                            stats.tileTriangles += QUAD_INDICES / 3;
                        }
                }
                ++stats.tiles;
            }
        }
    }

    void present() override {
        resolve();
        SDL_GL_SwapWindow(window_);
    }
    void resize(int w, int h) override {
        destroyTargets();
        createTargets(w, h);
    }
    void readPixels(int w, int h, std::vector<uint8_t>& rgb) override {
        resolve();
        rgb.resize(static_cast<size_t>(w) * h * 3);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, rgb.data());
    }

private:
    struct Mesh { GLuint vbo = 0; GLsizei quads = 0; };

    MeshId store(std::span<const uint32_t> v, GLsizei quads) {
        Mesh m;
        glCreateBuffers(1, &m.vbo);
        glNamedBufferStorage(m.vbo, static_cast<GLsizeiptr>(v.size_bytes()), v.data(), 0);
        m.quads = quads;
        if (!free_.empty()) {
            const MeshId id = free_.back();
            free_.pop_back();
            meshes_[id] = m;
            return id;
        }
        meshes_.push_back(m);
        return static_cast<MeshId>(meshes_.size() - 1);
    }

    // Свой буфер кадра: цвет и глубина с MSAA ×4, глубина - 32-битная плавающая (Р20); в окно - разрешением
    void createTargets(int w, int h) {
        width_ = w > 0 ? w : 1;
        height_ = h > 0 ? h : 1;
        glCreateRenderbuffers(1, &colorRb_);
        glNamedRenderbufferStorageMultisample(colorRb_, SAMPLES, GL_RGBA8, width_, height_);
        glCreateRenderbuffers(1, &depthRb_);
        glNamedRenderbufferStorageMultisample(depthRb_, SAMPLES, GL_DEPTH_COMPONENT32F, width_, height_);
        glCreateFramebuffers(1, &fbo_);
        glNamedFramebufferRenderbuffer(fbo_, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colorRb_);
        glNamedFramebufferRenderbuffer(fbo_, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthRb_);
        if (glCheckNamedFramebufferStatus(fbo_, GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            journal::error("opengl", "буфер кадра {}×{} не собран", width_, height_);
    }
    void destroyTargets() {
        if (fbo_) glDeleteFramebuffers(1, &fbo_);
        if (colorRb_) glDeleteRenderbuffers(1, &colorRb_);
        if (depthRb_) glDeleteRenderbuffers(1, &depthRb_);
        fbo_ = colorRb_ = depthRb_ = 0;
    }
    void resolve() {
        glBlitNamedFramebuffer(fbo_, 0, 0, 0, width_, height_, 0, 0, width_, height_, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    SDL_Window* window_;
    SDL_GLContext context_;
    GLuint program_ = 0, tileProgram_ = 0, vao_ = 0, tileVao_ = 0, ibo_ = 0, tileIbo_ = 0, ubo_ = 0;
    GLuint fbo_ = 0, colorRb_ = 0, depthRb_ = 0, maskTex_ = 0;
    int maskSide_ = 0;
    int width_ = 1, height_ = 1;
    std::vector<Mesh> meshes_;
    std::vector<MeshId> free_;
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
    int w = 1, h = 1;
    SDL_GetWindowSizeInPixels(window, &w, &h);
    auto backend = std::make_unique<GLBackend>(window, ctx);
    if (!backend->init(shaderDir, w, h)) return nullptr;
    return backend;
}

}  // namespace render
