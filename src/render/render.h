#pragma once
// Отрисовка глазами движка (Р8). Здесь нет ни одного типа OpenGL: движок записывает кадр в список
// команд, а путь отрисовки (сейчас - OpenGL, src/render/gl) его исполняет. Сетки в видеопамяти движок
// знает только по своему номеру (MeshId) - непрозрачный дескриптор.
#include <cstdint>
#include <memory>
#include <span>
#include <variant>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

struct SDL_Window;

namespace render {

struct Color { float r, g, b, a; };
using MeshId = uint32_t;
constexpr MeshId NO_MESH = 0;

struct CmdClear { Color color; };
struct CmdCamera { glm::mat4 viewProj; glm::vec3 sunDir; };
struct CmdDrawChunk { MeshId mesh; glm::ivec3 originVoxels; };
using Command = std::variant<CmdClear, CmdCamera, CmdDrawChunk>;

struct CommandList {
    std::vector<Command> commands;
    void clear(Color c) { commands.push_back(CmdClear{c}); }
    void camera(const glm::mat4& viewProj, glm::vec3 sun) { commands.push_back(CmdCamera{viewProj, sun}); }
    void drawChunk(MeshId m, glm::ivec3 origin) { commands.push_back(CmdDrawChunk{m, origin}); }
    void reset() { commands.clear(); }
};

struct FrameStats { int drawCalls = 0; int64_t quads = 0; };

class Backend {
public:
    virtual ~Backend() = default;
    // сетка куска: вершины по 4 на прямоугольник (см. world/mesher.h)
    virtual MeshId uploadChunkMesh(std::span<const uint32_t> vertices) = 0;
    virtual void execute(const CommandList& list, FrameStats& stats) = 0;
    virtual void present() = 0;
    virtual void resize(int width, int height) = 0;
    // снимок кадра (RGB, строки снизу вверх, как отдаёт видеокарта) - для проверки без человека у экрана
    virtual void readPixels(int width, int height, std::vector<uint8_t>& rgb) = 0;
};

// Путь отрисовки OpenGL 4.6. shaderDir - папка с собранными шейдерами (SPIR-V). nullptr - не удалось.
std::unique_ptr<Backend> createGL(SDL_Window* window, const char* shaderDir);

}  // namespace render
