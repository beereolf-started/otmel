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

GLuint loadProgram(const std::string& dir, const char* name, const char* fragName = nullptr) {
    const GLuint vs = loadSpirv(dir + "/" + name + ".vert.spv", GL_VERTEX_SHADER);
    const GLuint fs = loadSpirv(dir + "/" + (fragName ? fragName : name) + ".frag.spv", GL_FRAGMENT_SHADER);
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
    float fade[4];   // начало, конец растворения вдали; дымка: расстояние, доля неба
    float sky[4];
    float flow[4];   // Р43: поле течений - угол x, z, сторона (блоки)
    float time[4];   // Р43: секунды
    float simArea[4];   // Р45: зона автомата воды
    float invViewProj[16];   // Р51: из экрана в мир - толщина воды по лучу (глубина сцены под водой)
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
        dynProgram_ = loadProgram(shaderDir, "dynwater", "terrain");   // Р45: вода автомата
        if (!program_ || !tileProgram_ || !dynProgram_) return false;
        glCreateVertexArrays(1, &dynVao_);
        glEnableVertexArrayAttrib(dynVao_, 0);
        glVertexArrayAttribFormat(dynVao_, 0, 4, GL_FLOAT, GL_FALSE, 0);
        glVertexArrayAttribBinding(dynVao_, 0, 0);
        glCreateBuffers(1, &dynVbo_);

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

        // Узел плитки - пять целых (Р20, Р25, Р34): четыре - рельеф, пятое - поверхность воды
        glCreateVertexArrays(1, &tileVao_);
        glEnableVertexArrayAttrib(tileVao_, 0);
        glVertexArrayAttribIFormat(tileVao_, 0, 4, GL_UNSIGNED_INT, 0);
        glVertexArrayAttribBinding(tileVao_, 0, 0);
        glEnableVertexArrayAttrib(tileVao_, 1);
        glVertexArrayAttribIFormat(tileVao_, 1, 1, GL_UNSIGNED_INT, 16);
        glVertexArrayAttribBinding(tileVao_, 1, 0);
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
        const float still[2] = {0, 0};
        updateFlowField(1, {still, 2});    // пока поля нет - вода стоит
        return true;
    }

    MeshId uploadChunkMesh(std::span<const uint32_t> v, int waterQuads) override {
        if (v.empty()) return NO_MESH;
        return store(v, static_cast<GLsizei>(v.size() / 4), waterQuads);
    }

    MeshId uploadTile(std::span<const uint32_t> words, bool water) override { return store(words, 0, water ? 1 : 0); }

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

    void updateFlowField(int side, std::span<const float> v) override {
        if (side != flowSide_) {
            if (flowTex_) glDeleteTextures(1, &flowTex_);
            glCreateTextures(GL_TEXTURE_2D, 1, &flowTex_);
            glTextureStorage2D(flowTex_, 1, GL_RG32F, side, side);
            glTextureParameteri(flowTex_, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTextureParameteri(flowTex_, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTextureParameteri(flowTex_, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTextureParameteri(flowTex_, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            flowSide_ = side;
        }
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        glTextureSubImage2D(flowTex_, 0, 0, 0, side, side, GL_RG, GL_FLOAT, v.data());
    }

    void setSimWater(std::span<const float> v) override {
        dynCount_ = static_cast<GLsizei>(v.size() / 4);
        if (v.empty()) return;
        glNamedBufferData(dynVbo_, static_cast<GLsizeiptr>(v.size_bytes()), v.data(), GL_DYNAMIC_DRAW);
    }

    void execute(const CommandList& list, FrameStats& stats) override {
        stats = {};
        GLuint current = 0;
        water_.clear();
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
                u.fade[2] = cam->hazeDistance; u.fade[3] = cam->hazeMax;
                u.sky[0] = cam->sky.r; u.sky[1] = cam->sky.g; u.sky[2] = cam->sky.b; u.sky[3] = cam->sky.a;
                for (int i = 0; i < 4; ++i) u.flow[i] = cam->flowArea[i];
                u.time[0] = cam->time;
                for (int i = 0; i < 4; ++i) u.simArea[i] = cam->simArea[i];
                const glm::mat4 inv = glm::inverse(cam->viewProj);
                const float* mi = &inv[0][0];
                for (int i = 0; i < 16; ++i) u.invViewProj[i] = mi[i];
                simArea_ = cam->simArea;
                glNamedBufferSubData(ubo_, 0, sizeof(u), &u);
                glBindBufferBase(GL_UNIFORM_BUFFER, 0, ubo_);
            } else if (auto* d = std::get_if<CmdDrawChunk>(&cmd)) {
                const Mesh& m = meshes_[d->mesh];
                if (!m.vbo) continue;
                if (current != program_) { glUseProgram(program_); glBindVertexArray(vao_); current = program_; }
                if (m.water) water_.push_back(&cmd);
                stats.quads += m.quads;
                if (m.quads == m.water) continue;
                glProgramUniform3i(program_, 0, d->originVoxels.x, d->originVoxels.y, d->originVoxels.z);
                glVertexArrayVertexBuffer(vao_, 0, m.vbo, 0, 4);
                glDrawElements(GL_TRIANGLES, (m.quads - m.water) * 6, GL_UNSIGNED_INT, nullptr);
                ++stats.drawCalls;
            } else if (auto* t = std::get_if<CmdDrawTile>(&cmd)) {
                const Mesh& m = meshes_[t->mesh];
                if (!m.vbo) continue;
                if (current != tileProgram_) {
                    glUseProgram(tileProgram_);
                    glBindVertexArray(tileVao_);
                    glBindTextureUnit(0, maskTex_);
                    current = tileProgram_;
                }
                if (m.water) water_.push_back(&cmd);
                drawTile(*t, m, stats);
                ++stats.tiles;
            }
        }
        drawWater(stats);
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
    // water: у куска - сколько последних прямоугольников - вода; у плитки - 1, если в ней есть вода (Р34)
    struct Mesh { GLuint vbo = 0; GLsizei quads = 0, water = 0; };

    void drawTile(const CmdDrawTile& t, const Mesh& m, FrameStats& stats) {
        glProgramUniform2i(tileProgram_, 0, t.originBlocks.x, t.originBlocks.y);
        glProgramUniform1i(tileProgram_, 1, t.spacing);
        glProgramUniform2f(tileProgram_, 2, t.morphStart, t.morphEnd);
        glProgramUniform1i(tileProgram_, 3, t.level);
        glVertexArrayVertexBuffer(tileVao_, 0, m.vbo, 0, 20);
        if (t.quadrants == 0xF) {
            glDrawElements(GL_TRIANGLES, 4 * QUAD_INDICES, GL_UNSIGNED_SHORT, nullptr);
            ++stats.drawCalls;
            stats.tileTriangles += 4 * QUAD_INDICES / 3;
        } else {
            for (int q = 0; q < 4; ++q)
                if (t.quadrants >> q & 1) {
                    glDrawElements(GL_TRIANGLES, QUAD_INDICES, GL_UNSIGNED_SHORT,
                                   reinterpret_cast<const void*>(static_cast<uintptr_t>(q * QUAD_INDICES * 2)));
                    ++stats.drawCalls;
                    stats.tileTriangles += QUAD_INDICES / 3;
                }
        }
    }

    // Р34: вода - после всего непрозрачного, смешением «поверх» (a·вода + (1 - a)·что за ней). Глубину проверяет,
    // но не пишет: вода не заслоняет воду за собой. Порядок между кусками воды не сортируется - слоёв воды
    // друг за другом почти не бывает (край водопада над гладью).
    void drawWater(FrameStats& stats) {
        if (water_.empty() && dynCount_ == 0) return;
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        glProgramUniform1i(tileProgram_, 5, 1);
        // Р51: глубина сцены без воды - в текстуру (из MSAA - ближайшим образцом): по ней шейдер воды знает
        // толщину воды по лучу - мелко прозрачно, глубоко темно и мутно
        glBlitNamedFramebuffer(fbo_, depthFbo_, 0, 0, width_, height_, 0, 0, width_, height_, GL_DEPTH_BUFFER_BIT, GL_NEAREST);
        glBindTextureUnit(2, depthTex_);
        GLuint current = 0;
        for (const Command* cmd : water_) {
            if (auto* d = std::get_if<CmdDrawChunk>(cmd)) {
                const Mesh& m = meshes_[d->mesh];
                if (current != program_) {
                    glUseProgram(program_);
                    glBindVertexArray(vao_);
                    glBindTextureUnit(1, flowTex_);   // Р43: поле течений
                    current = program_;
                }
                // Р45: в зоне автомата воду кусков не рисуем - там вода автомата; граница зоны кратна 32 блокам,
                // кусок (2 блока) её не пересекает
                if (simArea_.w > 0) {
                    const float bx = float(d->originVoxels.x) / 16.0f, bz = float(d->originVoxels.z) / 16.0f;
                    if (bx >= simArea_.x && bz >= simArea_.y && bx + 2 <= simArea_.x + simArea_.z && bz + 2 <= simArea_.y + simArea_.z)
                        continue;
                }
                glProgramUniform3i(program_, 0, d->originVoxels.x, d->originVoxels.y, d->originVoxels.z);
                glVertexArrayVertexBuffer(vao_, 0, m.vbo, 0, 4);
                glDrawElements(GL_TRIANGLES, m.water * 6, GL_UNSIGNED_INT,
                               reinterpret_cast<const void*>(static_cast<uintptr_t>((m.quads - m.water) * 6 * 4)));
                ++stats.drawCalls;
            } else if (auto* t = std::get_if<CmdDrawTile>(cmd)) {
                if (current != tileProgram_) {
                    glUseProgram(tileProgram_);
                    glBindVertexArray(tileVao_);
                    glBindTextureUnit(0, maskTex_);
                    current = tileProgram_;
                }
                drawTile(*t, meshes_[t->mesh], stats);
            }
        }
        // Р45: вода автомата - поверх, тем же смешением; стенки водопадов видны с обеих сторон
        if (dynCount_ > 0) {
            glUseProgram(dynProgram_);
            glBindVertexArray(dynVao_);
            glBindTextureUnit(1, flowTex_);
            glVertexArrayVertexBuffer(dynVao_, 0, dynVbo_, 0, 16);
            glDisable(GL_CULL_FACE);
            glDrawArrays(GL_TRIANGLES, 0, dynCount_);
            glEnable(GL_CULL_FACE);
            ++stats.drawCalls;
        }
        glProgramUniform1i(tileProgram_, 5, 0);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }

    MeshId store(std::span<const uint32_t> v, GLsizei quads, GLsizei water) {
        Mesh m;
        glCreateBuffers(1, &m.vbo);
        glNamedBufferStorage(m.vbo, static_cast<GLsizeiptr>(v.size_bytes()), v.data(), 0);
        m.quads = quads;
        m.water = water;
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
        // Р51: копия глубины для воды - текстура без MSAA
        glCreateTextures(GL_TEXTURE_2D, 1, &depthTex_);
        glTextureStorage2D(depthTex_, 1, GL_DEPTH_COMPONENT32F, width_, height_);
        glTextureParameteri(depthTex_, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTextureParameteri(depthTex_, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTextureParameteri(depthTex_, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTextureParameteri(depthTex_, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glCreateFramebuffers(1, &depthFbo_);
        glNamedFramebufferTexture(depthFbo_, GL_DEPTH_ATTACHMENT, depthTex_, 0);
        if (glCheckNamedFramebufferStatus(depthFbo_, GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            journal::error("opengl", "копия глубины {}×{} не собрана", width_, height_);
    }
    void destroyTargets() {
        if (fbo_) glDeleteFramebuffers(1, &fbo_);
        if (colorRb_) glDeleteRenderbuffers(1, &colorRb_);
        if (depthRb_) glDeleteRenderbuffers(1, &depthRb_);
        if (depthFbo_) glDeleteFramebuffers(1, &depthFbo_);
        if (depthTex_) glDeleteTextures(1, &depthTex_);
        fbo_ = colorRb_ = depthRb_ = depthFbo_ = depthTex_ = 0;
    }
    void resolve() {
        glBlitNamedFramebuffer(fbo_, 0, 0, 0, width_, height_, 0, 0, width_, height_, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    SDL_Window* window_;
    SDL_GLContext context_;
    GLuint program_ = 0, tileProgram_ = 0, vao_ = 0, tileVao_ = 0, ibo_ = 0, tileIbo_ = 0, ubo_ = 0;
    GLuint fbo_ = 0, colorRb_ = 0, depthRb_ = 0, maskTex_ = 0, flowTex_ = 0;
    GLuint depthFbo_ = 0, depthTex_ = 0;   // Р51: копия глубины сцены для воды
    GLuint dynProgram_ = 0, dynVao_ = 0, dynVbo_ = 0;   // Р45: вода автомата
    GLsizei dynCount_ = 0;
    glm::vec4 simArea_{0};
    int flowSide_ = 0;
    int maskSide_ = 0;
    int width_ = 1, height_ = 1;
    std::vector<Mesh> meshes_;
    std::vector<MeshId> free_;
    std::vector<const Command*> water_;   // кадр: что рисовать вторым, прозрачным проходом (Р34)
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
