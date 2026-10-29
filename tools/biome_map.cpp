// Карта мира сверху - тем же кодом генерации, что и мир (не прототип): высоты с отмывкой рельефа и покров,
// вторым снимком - веса видов рельефа. Плюс доли площади и замеры. Р19.
//   otmel_map --out путь_без_расширения --center X Z --bpp блоков_на_точку --size точек
#define _CRT_SECURE_NO_WARNINGS
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "world/terrain.h"

namespace {

void saveBmp(const std::string& path, int w, int h, const std::vector<uint8_t>& rgb) {
    const int row = (w * 3 + 3) & ~3;
    const uint32_t size = 54 + uint32_t(row) * h;
    uint8_t hdr[54] = {'B', 'M'};
    auto put = [&](int at, uint32_t v) { std::memcpy(hdr + at, &v, 4); };
    put(2, size); put(10, 54); put(14, 40); put(18, uint32_t(w)); put(22, uint32_t(h));
    hdr[26] = 1; hdr[28] = 24;
    FILE* f = std::fopen(path.c_str(), "wb");
    std::fwrite(hdr, 1, 54, f);
    std::vector<uint8_t> line(row, 0);
    for (int y = h - 1; y >= 0; --y) {   // BMP - снизу вверх, BGR
        for (int x = 0; x < w; ++x) {
            const uint8_t* p = &rgb[(size_t(y) * w + x) * 3];
            line[x * 3] = p[2]; line[x * 3 + 1] = p[1]; line[x * 3 + 2] = p[0];
        }
        std::fwrite(line.data(), 1, row, f);
    }
    std::fclose(f);
}

}  // namespace

int main(int argc, char** argv) {
    std::string out = "map";
    int cx = 0, cz = 0, bpp = 4, size = 1024, style = 2;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--out" && i + 1 < argc) out = argv[++i];
        else if (a == "--center" && i + 2 < argc) { cx = std::stoi(argv[++i]); cz = std::stoi(argv[++i]); }
        else if (a == "--bpp" && i + 1 < argc) bpp = std::stoi(argv[++i]);
        else if (a == "--size" && i + 1 < argc) size = std::stoi(argv[++i]);
        else if (a == "--style" && i + 1 < argc) style = std::stoi(argv[++i]);
    }
    world::TerrainParams p;
    p.mountainStyle = style;
    const int n = size;
    std::vector<world::Relief> rel(size_t(n) * n);
    std::vector<world::Cover> cov(size_t(n) * n);
    // точка карты -> воксель: в центре клетки bpp×bpp блоков
    auto vox = [&](int center, int i) { return (center + (i - n / 2) * bpp) * world::VOXELS_PER_BLOCK + bpp * 8; };

    const auto t0 = std::chrono::steady_clock::now();
    std::atomic<int> nextRow{0};
    auto work = [&] {
        for (int z; (z = nextRow++) < n;)
            for (int x = 0; x < n; ++x) {
                const int32_t wx = vox(cx, x), wz = vox(cz, z);
                const world::Relief r = world::reliefAt(p, wx, wz);
                rel[size_t(z) * n + x] = r;
                cov[size_t(z) * n + x] = world::coverAt(p, world::climateAt(p, wx, wz), r.height);
            }
    };
    std::vector<std::thread> pool;
    const unsigned threads = std::max(1u, std::thread::hardware_concurrency());
    for (unsigned t = 0; t < threads; ++t) pool.emplace_back(work);
    for (auto& t : pool) t.join();
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    // 1) рельеф: цвет покрова × отмывка (свет с северо-запада, как на картах) × лёгкое затемнение низин
    std::vector<uint8_t> img(size_t(n) * n * 3), zones(size_t(n) * n * 3);
    const float colors[6][3] = {{1, 0, 1}, {0.47f, 0.47f, 0.5f}, {0.45f, 0.32f, 0.2f}, {0.36f, 0.58f, 0.25f},
                                {0.85f, 0.8f, 0.55f}, {0.93f, 0.95f, 0.98f}};
    int64_t cnt[3] = {}, mixed = 0, snow = 0, sand = 0, sea = 0;
    int32_t hmin = INT32_MAX, hmax = INT32_MIN;
    int topX = 0, topZ = 0;
    for (int z = 0; z < n; ++z)
        for (int x = 0; x < n; ++x) {
            const size_t i = size_t(z) * n + x;
            const world::Relief& r = rel[i];
            hmin = std::min(hmin, r.height);
            if (r.height > hmax) { hmax = r.height; topX = cx + (x - n / 2) * bpp; topZ = cz + (z - n / 2) * bpp; }
            const auto H = [&](int xx, int zz) {
                return float(rel[size_t(std::clamp(zz, 0, n - 1)) * n + std::clamp(xx, 0, n - 1)].height);
            };
            const float step = float(bpp * world::VOXELS_PER_BLOCK) * 2.0f;
            const float sx = (H(x + 1, z) - H(x - 1, z)) / step, sz = (H(x, z + 1) - H(x, z - 1)) / step;   // свет с северо-запада
            const float light = std::clamp(0.72f + 0.9f * (sx + sz), 0.25f, 1.25f);
            const float alt = std::clamp(0.8f + 0.2f * float(r.height) / float(160 * world::VOXELS_PER_BLOCK), 0.6f, 1.15f);
            const float water[3] = {0.16f, 0.32f, 0.52f};
            const bool wet = r.water > r.height;   // Р25, Р29: под водой (море, река, озеро) - синий, темнее с глубиной
            const float depthK = wet ? std::clamp(1.0f - float(r.water - r.height) / (60.0f * world::VOXELS_PER_BLOCK), 0.45f, 1.0f) : 1.0f;
            const float* c = wet ? water : colors[std::min<int>(cov[i].top, 5)];
            for (int k = 0; k < 3; ++k)
                img[i * 3 + k] = uint8_t(std::clamp(wet ? c[k] * depthK * (0.6f + 0.4f * light) * 255.0f : c[k] * light * alt * 255.0f, 0.0f, 255.0f));
            if (wet) ++sea;
            // 2) зоны: равнины - зелёный, холмы - охра, горы - бурый, смешанные - в пропорции весов
            const float wp = r.wPlains / 65536.0f, wh = r.wHills / 65536.0f, wm = r.wMount / 65536.0f;
            const float zc[3][3] = {{0.45f, 0.7f, 0.35f}, {0.85f, 0.7f, 0.35f}, {0.45f, 0.3f, 0.25f}};
            for (int k = 0; k < 3; ++k)
                zones[i * 3 + k] = uint8_t(std::clamp((wp * zc[0][k] + wh * zc[1][k] + wm * zc[2][k]) * light * 255.0f, 0.0f, 255.0f));
            const int dom = r.wMount >= r.wHills && r.wMount >= r.wPlains ? 2 : (r.wHills >= r.wPlains ? 1 : 0);
            ++cnt[dom];
            if (r.wPlains != 65536 && r.wHills != 65536 && r.wMount != 65536) ++mixed;
            if (cov[i].top == world::SNOW) ++snow;
            if (cov[i].top == world::SAND) ++sand;
        }
    saveBmp(out + "_relief.bmp", n, n, img);
    saveBmp(out + "_zones.bmp", n, n, zones);
    const double all = double(n) * n;
    std::printf("карта %d×%d точек по %d блоков (%d×%d блоков), центр (%d, %d)\n", n, n, bpp, n * bpp, n * bpp, cx, cz);
    std::printf("преобладают: равнины %.1f%%, холмы %.1f%%, горы %.1f%%; в переходе (смесь) %.1f%%\n",
                100 * cnt[0] / all, 100 * cnt[1] / all, 100 * cnt[2] / all, 100 * mixed / all);
    std::printf("море: %.1f%%\n", 100 * sea / all);
    std::printf("покров: снег %.1f%%, песок %.1f%%; высоты %.1f .. %.1f блоков\n", 100 * snow / all, 100 * sand / all,
                hmin / 16.0, hmax / 16.0);
    std::printf("высшая точка - блок (%d, %d)\n", topX, topZ);
    std::printf("расчёт: %.0f мс на %u потоках, %.2f мкс на столбец (на одном потоке)\n", ms, threads,
                ms * 1000.0 * threads / all);
    return 0;
}
