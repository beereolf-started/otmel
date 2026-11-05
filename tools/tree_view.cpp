// Р91: взгляд на дерево сбоку - без игры. Три возраста, летом и зимой, в одну картинку PPM.
// Запуск: otmel_treeview out.ppm [семя]
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "world/tree.h"

using namespace world;

int main(int argc, char** argv) {
    const char* path = argc > 1 ? argv[1] : "trees.ppm";
    const uint32_t seed0 = argc > 2 ? uint32_t(std::strtoul(argv[2], nullptr, 10)) : 1u;
    constexpr int PANEL = 260, COLS = int(TreeSpecies::COUNT), ROWS = 2;
    const int W = PANEL * COLS, Hh = PANEL * ROWS;
    std::vector<unsigned char> img(size_t(W) * Hh * 3, 0);
    for (size_t i = 0; i < img.size(); i += 3) { img[i] = 170; img[i + 1] = 200; img[i + 2] = 230; }
    std::vector<Material> vox(32 * 32 * 32);
    for (int row = 0; row < ROWS; ++row)
        for (int col = 0; col < COLS; ++col) {
            const TreeAge age = TreeAge::ADULT;
            const TreeSpecies species = TreeSpecies(col);
            const uint32_t seed = seed0 * 7919u + uint32_t(col) * 104729u;
            const bool winter = row == 1;
            auto t = growTree(seed, age, species);
            // вид сбоку: x вправо, y вверх; глубина z - ближнее побеждает
            const int scale = age == TreeAge::YOUNG ? 1 : (age == TreeAge::ADULT ? 2 : 2);
            // общий масштаб для всех возрастов: 30 м на клетку - видно, кто крупнее
            const float k = float(PANEL - 20) / (24.0f * 16.0f);   // точек на мелкий воксель (24 м на клетку)
            std::vector<float> zb(size_t(PANEL) * PANEL, 1e9f);
            int lo[3], hi[3];
            treeChunkRange(*t, scale, lo, hi);
            const TreeIndex ix = treeIndex(*t, scale);
            const float cxm = (t->lo[0] + t->hi[0]) * 0.5f;
            for (int cz = lo[2]; cz <= hi[2]; ++cz)
                for (int cy = lo[1]; cy <= hi[1]; ++cy)
                    for (int cx = lo[0]; cx <= hi[0]; ++cx) {
                        if (!treeChunk(*t, ix, cx, cy, cz, true, !winter, vox.data())) continue;
                        for (int y = 0; y < 32; ++y)
                            for (int z = 0; z < 32; ++z)
                                for (int x = 0; x < 32; ++x) {
                                    const Material m = vox[(size_t(y) * 32 + z) * 32 + x];
                                    if (m == AIR) continue;
                                    const float fx = float((cx * 32 + x) * scale), fy = float((cy * 32 + y) * scale), fz = float((cz * 32 + z) * scale);
                                    const int px0 = int((fx - cxm) * k + PANEL / 2), py0 = int(PANEL - 10 - fy * k);
                                    const int sz = std::max(1, int(scale * k + 0.99f));
                                    for (int dy = 0; dy < sz; ++dy)
                                        for (int dx = 0; dx < sz; ++dx) {
                                            const int px = px0 + dx, py = py0 - dy;
                                            if (px < 0 || py < 0 || px >= PANEL || py >= PANEL) continue;
                                            float& d = zb[size_t(py) * PANEL + px];
                                            if (fz >= d) continue;
                                            d = fz;
                                            const float shade = 0.65f + 0.35f * std::clamp(1.0f - (fz - t->lo[2]) / std::max(1.0f, t->hi[2] - t->lo[2]), 0.0f, 1.0f);
                                            int r, g, b;
                                            static const int PAL[][3] = {{60, 45, 35}};
                                            (void)PAL;
                                            if (m == LEAVES || m == LEAF_LIGHT) { r = 70; g = 120; b = 40; }
                                            else if (m == LEAVES2 || m == LEAF_LIGHT2) { r = 95; g = 145; b = 50; }
                                            else if (m == LEAF_BIRCH || m == LEAF_BIRCH2) { r = 110; g = 165; b = 60; }
                                            else if (m == NEEDLE || m == NEEDLE2) { r = 30; g = 75; b = 45; }
                                            else if (m == LEAF_WILLOW || m == LEAF_WILLOW2) { r = 120; g = 145; b = 100; }
                                            else if (m == BERRY) { r = 200; g = 30; b = 30; }
                                            else if (m == BIRCH_BARK) { r = 220; g = 218; b = 208; }
                                            else if (m == BIRCH_BARK2) { r = 30; g = 30; b = 30; }
                                            else if (m == PINE_BARK) { r = 140; g = 85; b = 50; }
                                            else if (m == BARK2) { r = 90; g = 80; b = 70; }
                                            else if (m == WOOD) { r = 170; g = 130; b = 80; }
                                            else { r = 60; g = 45; b = 35; }
                                            unsigned char* o = &img[(size_t(row * PANEL + py) * W + col * PANEL + px) * 3];
                                            o[0] = (unsigned char)(r * shade); o[1] = (unsigned char)(g * shade); o[2] = (unsigned char)(b * shade);
                                        }
                                }
                    }
            // земля
            for (int px = 0; px < PANEL; ++px)
                for (int py = PANEL - 10; py < PANEL; ++py) {
                    unsigned char* o = &img[(size_t(row * PANEL + py) * W + col * PANEL + px) * 3];
                    o[0] = 90; o[1] = 120; o[2] = 50;
                }
            std::printf("%s ", speciesName(species));
            std::printf("%s %s: высота %.1f м, крона %.1f м, ствол %.0f см, узлов %zu, ветвей %zu, листвы %zu, корней %d\n",
                        age == TreeAge::YOUNG ? "молодой" : age == TreeAge::ADULT ? "взрослый" : "вековой", winter ? "зима" : "лето",
                        t->height / 16.0f, 2.0f * t->crownR / 16.0f, t->nodes[0].r * 2.0f / 16.0f * 100.0f, t->nodes.size(),
                        t->branches.size(), t->leaves.size(), t->roots);
        }
    FILE* f = std::fopen(path, "wb");
    std::fprintf(f, "P6 %d %d 255\n", W, Hh);
    std::fwrite(img.data(), 1, img.size(), f);
    std::fclose(f);
}
