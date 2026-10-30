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
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

struct SDL_Window;

namespace render {

struct Color { float r, g, b, a; };
using MeshId = uint32_t;
constexpr MeshId NO_MESH = 0;

struct CmdClear { Color color; };
// Камера кадра. Проекция - с обратной глубиной (Р20): ближняя плоскость -> 1, дальняя -> 0.
struct CmdCamera {
    glm::mat4 viewProj;
    glm::vec3 sunDir;
    glm::vec3 eye;            // положение камеры, блоки
    glm::vec4 voxelZone;      // окно маски вокселей: minX, minZ (блоки), столбцов по стороне, блоков в столбце (Р21)
    float fadeStart, fadeEnd; // дальний край видимости: рельеф растворяется в небе
    float hazeDistance;       // воздушная дымка: на этом расстоянии - 63% от hazeMax (1 - e^-1)
    float hazeMax;            // сколько неба примешивается вдали при полной дымке
    Color sky;
    float time = 0;           // Р43: секунды с начала - рябь, пена, падающая вода
    glm::vec4 flowArea{0};    // Р43: поле течений: угол x, z (блоки), сторона (блоки)
};
struct CmdDrawChunk { MeshId mesh; glm::ivec3 originVoxels; };
// Плитка дальнего уровня (Р20): начало в блоках, шаг узлов в блоках, где идёт морфинг, какие четверти
struct CmdDrawTile {
    MeshId mesh;
    glm::ivec2 originBlocks;
    int spacing, level;
    float morphStart, morphEnd;
    uint8_t quadrants;
};
using Command = std::variant<CmdClear, CmdCamera, CmdDrawChunk, CmdDrawTile>;

struct CommandList {
    std::vector<Command> commands;
    void clear(Color c) { commands.push_back(CmdClear{c}); }
    void camera(const CmdCamera& c) { commands.push_back(c); }
    void drawChunk(MeshId m, glm::ivec3 origin) { commands.push_back(CmdDrawChunk{m, origin}); }
    void drawTile(const CmdDrawTile& t) { commands.push_back(t); }
    void reset() { commands.clear(); }
};

struct FrameStats { int drawCalls = 0, tiles = 0; int64_t quads = 0, tileTriangles = 0; };

class Backend {
public:
    virtual ~Backend() = default;
    // сетка куска: вершины по 4 на прямоугольник (см. world/mesher.h); последние waterQuads - вода (Р34)
    virtual MeshId uploadChunkMesh(std::span<const uint32_t> vertices, int waterQuads) = 0;
    // плитка дальнего уровня: 33×33 узла по 5 чисел (см. world/lod.h); water - есть ли в ней вода (Р34)
    virtual MeshId uploadTile(std::span<const uint32_t> words, bool water) = 0;
    virtual void destroyMesh(MeshId id) = 0;
    // маска готовых столбцов вокселей (Р21): side×side байт, строка - по z; над единицами плитки не рисуются
    virtual void updateVoxelMask(int side, std::span<const uint8_t> mask) = 0;
    // Р43: поле течений side×side блоков, 2 числа на блок (скорость x, z, блоки/с)
    virtual void updateFlowField(int side, std::span<const float> velocity) = 0;
    // параметр отладки шейдеров: 1 - раскрасить плитки по уровням
    virtual void setDebug(int mode) = 0;
    virtual void execute(const CommandList& list, FrameStats& stats) = 0;
    virtual void present() = 0;
    virtual void resize(int width, int height) = 0;
    // снимок кадра (RGB, строки снизу вверх, как отдаёт видеокарта) - для проверки без человека у экрана
    virtual void readPixels(int width, int height, std::vector<uint8_t>& rgb) = 0;
};

// Путь отрисовки OpenGL 4.6. shaderDir - папка с собранными шейдерами (SPIR-V). nullptr - не удалось.
std::unique_ptr<Backend> createGL(SDL_Window* window, const char* shaderDir);

}  // namespace render
