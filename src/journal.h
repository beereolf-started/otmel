#pragma once
// Журнал (Р6): уровни и подсистемы. Пишет в консоль и в otmel.log рядом с программой (Р64: дописывает, у запуска -
// дата и время; при падении - цепочка вызовов и дамп otmel.dmp).
#include <format>
#include <string_view>

namespace journal {

enum class Level { Debug, Info, Warn, Error };

void write(Level level, std::string_view subsystem, std::string_view text);
// Р64: перехват падения - в журнал код, адрес и цепочку вызовов, рядом - дамп памяти
void installCrashHandler();

template <class... A>
void info(std::string_view sub, std::format_string<A...> f, A&&... a) { write(Level::Info, sub, std::format(f, std::forward<A>(a)...)); }
template <class... A>
void warn(std::string_view sub, std::format_string<A...> f, A&&... a) { write(Level::Warn, sub, std::format(f, std::forward<A>(a)...)); }
template <class... A>
void error(std::string_view sub, std::format_string<A...> f, A&&... a) { write(Level::Error, sub, std::format(f, std::forward<A>(a)...)); }
template <class... A>
void debug(std::string_view sub, std::format_string<A...> f, A&&... a) { write(Level::Debug, sub, std::format(f, std::forward<A>(a)...)); }

}  // namespace journal
