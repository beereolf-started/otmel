#pragma once
// Отрисовка глазами движка (Р8). Здесь нет ни одного типа OpenGL: движок записывает кадр в список
// команд, а путь отрисовки (сейчас - OpenGL, src/render/gl) его исполняет. Так переход на Vulkan
// - это второй путь отрисовки, а не переписывание движка.
#include <memory>
#include <variant>
#include <vector>

struct SDL_Window;

namespace render {

struct Color { float r, g, b, a; };

// Команды кадра. Пока одна - залить экран цветом; дальше добавятся «нарисовать кусок» и прочие.
struct CmdClear { Color color; };
using Command = std::variant<CmdClear>;

struct CommandList {
    std::vector<Command> commands;
    void clear(Color c) { commands.push_back(CmdClear{c}); }
    void reset() { commands.clear(); }
};

class Backend {
public:
    virtual ~Backend() = default;
    virtual void execute(const CommandList& list) = 0;   // исполнить список команд
    virtual void present() = 0;                          // показать готовый кадр
};

// Путь отрисовки OpenGL 4.6. nullptr - не удалось (причина в журнале).
std::unique_ptr<Backend> createGL(SDL_Window* window);

}  // namespace render
