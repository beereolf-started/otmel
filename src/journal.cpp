#include "journal.h"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <mutex>
#include <string>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <dbghelp.h>

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

// папка программы: журнал и дамп - всегда рядом с otmel.exe, откуда бы её ни запустили
std::wstring exeDir() {
    wchar_t buf[MAX_PATH];
    const DWORD n = GetModuleFileNameW(nullptr, buf, MAX_PATH);
    std::wstring p(buf, n);
    const size_t s = p.find_last_of(L"\\/");
    return s == std::wstring::npos ? L"" : p.substr(0, s + 1);
}

// Р64: журнал дописывается (не затирается): у каждого запуска - строка с датой и временем
void open() {
    if (g_file) return;
    _wfopen_s(&g_file, (exeDir() + L"otmel.log").c_str(), L"a");
    if (!g_file) return;
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_s(&tm, &t);
    char when[64];
    std::strftime(when, sizeof(when), "%Y-%m-%d %H:%M:%S", &tm);
    std::fprintf(g_file, "\n===== запуск %s =====\n", when);
    std::fflush(g_file);
}

// Р64: падение - код, адрес, цепочка вызовов с файлами и строками (по PDB рядом), дамп памяти otmel.dmp
LONG WINAPI onCrash(EXCEPTION_POINTERS* e) {
    const HANDLE proc = GetCurrentProcess();
    SymSetOptions(SYMOPT_LOAD_LINES | SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS);
    SymInitialize(proc, nullptr, TRUE);
    const EXCEPTION_RECORD& r = *e->ExceptionRecord;
    std::string text = std::format("ПАДЕНИЕ: код 0x{:08X}, адрес 0x{:X}", uint32_t(r.ExceptionCode), uintptr_t(r.ExceptionAddress));
    if (r.ExceptionCode == EXCEPTION_ACCESS_VIOLATION && r.NumberParameters >= 2)
        text += std::format(" ({} по адресу 0x{:X})", r.ExceptionInformation[0] == 0 ? "чтение" : r.ExceptionInformation[0] == 1 ? "запись" : "исполнение",
                            r.ExceptionInformation[1]);
    write(Level::Error, "падение", text);
    // цепочка вызовов - от места падения
    CONTEXT ctx = *e->ContextRecord;
    STACKFRAME64 f{};
    f.AddrPC.Offset = ctx.Rip; f.AddrPC.Mode = AddrModeFlat;
    f.AddrFrame.Offset = ctx.Rbp; f.AddrFrame.Mode = AddrModeFlat;
    f.AddrStack.Offset = ctx.Rsp; f.AddrStack.Mode = AddrModeFlat;
    for (int i = 0; i < 32 && StackWalk64(IMAGE_FILE_MACHINE_AMD64, proc, GetCurrentThread(), &f, &ctx, nullptr,
                                          SymFunctionTableAccess64, SymGetModuleBase64, nullptr); ++i) {
        alignas(SYMBOL_INFO) char sb[sizeof(SYMBOL_INFO) + 256];
        auto* sym = reinterpret_cast<SYMBOL_INFO*>(sb);
        sym->SizeOfStruct = sizeof(SYMBOL_INFO);
        sym->MaxNameLen = 255;
        DWORD64 d = 0;
        IMAGEHLP_LINE64 line{};
        line.SizeOfStruct = sizeof(line);
        DWORD ld = 0;
        const bool hasSym = SymFromAddr(proc, f.AddrPC.Offset, &d, sym);
        const bool hasLine = SymGetLineFromAddr64(proc, f.AddrPC.Offset, &ld, &line);
        write(Level::Error, "падение", std::format("  #{} {} {}:{}", i, hasSym ? sym->Name : "?", hasLine ? line.FileName : "?", hasLine ? line.LineNumber : 0));
    }
    const HANDLE dump = CreateFileW((exeDir() + L"otmel.dmp").c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (dump != INVALID_HANDLE_VALUE) {
        MINIDUMP_EXCEPTION_INFORMATION mi{GetCurrentThreadId(), e, FALSE};
        MiniDumpWriteDump(proc, GetCurrentProcessId(), dump, MiniDumpWithIndirectlyReferencedMemory, &mi, nullptr, nullptr);
        CloseHandle(dump);
        write(Level::Error, "падение", "дамп памяти - otmel.dmp рядом с программой");
    }
    return EXCEPTION_EXECUTE_HANDLER;
}
}  // namespace

void write(Level level, std::string_view subsystem, std::string_view text) {
    // время от запуска в миллисекундах: по нему видно, что сколько заняло
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - g_start).count();
    std::lock_guard guard(g_lock);
    open();
    const std::string line = std::format("[{:>8} мс] [{}] [{}] {}\n", ms, levelName(level), subsystem, text);
    fputs(line.c_str(), stdout);
    if (g_file) { fputs(line.c_str(), g_file); fflush(g_file); }
}

void installCrashHandler() { SetUnhandledExceptionFilter(onCrash); }

}  // namespace journal
