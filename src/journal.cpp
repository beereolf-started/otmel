#include "journal.h"

#include <chrono>
#include <cstdio>
#include <mutex>

namespace journal {

namespace {
std::mutex g_lock;
FILE* g_file = nullptr;
const auto g_start = std::chrono::steady_clock::now();

const char* levelName(Level l) {
    switch (l) {
        case Level::Debug: return "отладка";
        case Level::Info: return "сведения";
        case Level::Warn: return "ВНИМАНИЕ";
        case Level::Error: return "ОШИБКА";
    }
    return "?";
}
}  // namespace

void write(Level level, std::string_view subsystem, std::string_view text) {
    // время от запуска в миллисекундах: по нему видно, что сколько заняло
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - g_start).count();
    std::lock_guard guard(g_lock);
    if (!g_file) fopen_s(&g_file, "otmel.log", "w");
    const std::string line = std::format("[{:>8} мс] [{}] [{}] {}\n", ms, levelName(level), subsystem, text);
    fputs(line.c_str(), stdout);
    if (g_file) { fputs(line.c_str(), g_file); fflush(g_file); }
}

}  // namespace journal
