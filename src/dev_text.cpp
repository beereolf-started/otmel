// Рисование строк дебаг-меню средствами системы (dev_text.h).
#include "dev_text.h"

#include <algorithm>

#ifndef NOMINMAX
#define NOMINMAX            // иначе макросы min и max из windows.h перебивают std::min
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

namespace dev {
namespace {

constexpr int PAD = 8;                       // поля панели, точки
constexpr uint8_t BG_R = 16, BG_G = 18, BG_B = 24, BG_A = 205;   // подложка
constexpr COLORREF TEXT_COLOR = RGB(228, 232, 238);

std::wstring wide(const std::string& s) {
    if (s.empty()) return {};
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), int(s.size()), nullptr, 0);
    std::wstring w(size_t(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), int(s.size()), w.data(), n);
    return w;
}

}  // namespace

bool renderLines(const std::vector<std::string>& lines, int fontPx, TextImage& out) {
    out.w = out.h = 0;
    out.rgba.clear();
    if (lines.empty()) return false;

    HDC screen = GetDC(nullptr);
    HDC dc = CreateCompatibleDC(screen);
    ReleaseDC(nullptr, screen);
    if (!dc) return false;

    // моноширинный системный шрифт: Consolas есть везде начиная с Vista, запасной - Courier New
    HFONT font = CreateFontW(-fontPx, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
                             OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
                             FIXED_PITCH | FF_MODERN, L"Consolas");
    if (!font) font = CreateFontW(-fontPx, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
                                  OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
                                  FIXED_PITCH | FF_MODERN, L"Courier New");
    if (!font) { DeleteDC(dc); return false; }
    HGDIOBJ oldFont = SelectObject(dc, font);

    // размер панели: самая длинная строка и высота строки
    TEXTMETRICW tm{};
    GetTextMetricsW(dc, &tm);
    const int lineH = tm.tmHeight + 2;
    int widest = 0;
    std::vector<std::wstring> wl;
    wl.reserve(lines.size());
    for (const std::string& s : lines) {
        wl.push_back(wide(s));
        SIZE sz{};
        GetTextExtentPoint32W(dc, wl.back().c_str(), int(wl.back().size()), &sz);
        widest = std::max<int>(widest, sz.cx);
    }
    const int w = std::min(widest + PAD * 2, 2048), h = int(wl.size()) * lineH + PAD * 2;

    BITMAPINFO bi{};
    bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bi.bmiHeader.biWidth = w;
    bi.bmiHeader.biHeight = -h;                 // сверху вниз
    bi.bmiHeader.biPlanes = 1;
    bi.bmiHeader.biBitCount = 32;
    bi.bmiHeader.biCompression = BI_RGB;
    void* bits = nullptr;
    HBITMAP bmp = CreateDIBSection(dc, &bi, DIB_RGB_COLORS, &bits, nullptr, 0);
    if (!bmp) { SelectObject(dc, oldFont); DeleteObject(font); DeleteDC(dc); return false; }
    HGDIOBJ oldBmp = SelectObject(dc, bmp);

    // подложка: GDI не пишет прозрачность, поэтому кладём её сами, а потом ставим одну на всю панель
    auto* px = static_cast<uint8_t*>(bits);
    for (int i = 0; i < w * h; ++i) {           // DIB хранит BGRA
        px[i * 4 + 0] = BG_B;
        px[i * 4 + 1] = BG_G;
        px[i * 4 + 2] = BG_R;
        px[i * 4 + 3] = 255;
    }
    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, TEXT_COLOR);
    for (size_t i = 0; i < wl.size(); ++i)
        TextOutW(dc, PAD, PAD + int(i) * lineH, wl[i].c_str(), int(wl[i].size()));
    GdiFlush();

    out.w = w;
    out.h = h;
    out.rgba.resize(size_t(w) * h * 4);
    for (int i = 0; i < w * h; ++i) {           // BGRA -> RGBA, прозрачность - одна на панель
        out.rgba[size_t(i) * 4 + 0] = px[i * 4 + 2];
        out.rgba[size_t(i) * 4 + 1] = px[i * 4 + 1];
        out.rgba[size_t(i) * 4 + 2] = px[i * 4 + 0];
        out.rgba[size_t(i) * 4 + 3] = BG_A;
    }

    SelectObject(dc, oldBmp);
    DeleteObject(bmp);
    SelectObject(dc, oldFont);
    DeleteObject(font);
    DeleteDC(dc);
    return true;
}

}  // namespace dev
