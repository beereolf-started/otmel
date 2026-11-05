#pragma once
// Кусок мира (Р14, Р17): 32×32×32 вокселя, плотный массив номеров материала по 2 байта (64 КБ).
// Воксель - 1/16 блока (Р13), кусок - 2×2×2 блока. Координаты куска - в кусках: (cx, cy, cz).
#include <array>
#include <cstdint>

namespace world {

constexpr int CHUNK = 32;                 // вокселей в куске по стороне
constexpr int CHUNK_LOG2 = 5;
constexpr int VOXELS_PER_BLOCK = 16;      // Р13

using Material = uint16_t;
enum : Material { AIR = 0, STONE = 1, DIRT = 2, GRASS = 3, SAND = 4, SNOW = 5, WATER = 6,
                    BARK = 7, WOOD = 8, LEAVES = 9, LEAVES2 = 10,   // Р53: дерево - кора, древесина, листва (два оттенка)
                    BARK2 = 11,                      // Р71: кора в трещине - темнее
                    ROCK_FIRST = 12, ROCK_LAST = 47, // Р77, Р78: 36 пород (world::Rock), материал = ROCK_FIRST + порода
                    // Р82: обрубленный дёрн - бок блока травы там, где до соседней поверхности два
                    // блока и больше: земля с полоской травы сверху. Где ступенька в один блок,
                    // бок остаётся сплошной травой (GRASS), как в TFC.
                    GRASS_SIDE = 48,
                    // Р82: бок блока под снежной шапкой - земля с полоской СНЕГА сверху. Иначе под
                    // снегом по склону шла бы зелёная лесенка дёрна.
                    GRASS_SNOWY_SIDE = 49,
                    // Р87: именованные почвы (world::soil_table) - материал = SOIL_FIRST + номер.
                    // Это ТЕЛО земли под покровом, а не верхний воксель: сверху лежит дёрн, песок
                    // или снег, а вот чем именно сложена толща - песком, суглинком, глиной, торфом -
                    // видно в обрыве и в выкопанной яме.
                    SOIL_FIRST = 50, SOIL_LAST = 58,
                    // Р88: рудные минералы (world::ores) - материал = ORE_FIRST + номер. Руда не
                    // отдельный блок, а вкрапления в породе: рисунок - накладка поверх камня.
                    // Р88: на минерал по три оттенка палитры - тёмный, основной и блестящий
                    ORE_FIRST = 59, ORE_LAST = 118,
                    // Р89: дёрн по почвам - у каждой почвы своя трава: верх, бок (обрубленный дёрн) и
                    // бок под снежной шапкой. Номер почвы - как у SOIL_FIRST.
                    GRASS_FIRST = 119, GRASS_LAST = 127,
                    GRASS_SIDE_FIRST = 128, GRASS_SNOWY_FIRST = 137,
                    IRON = 146,                     // Р92: железо инструментов (палитра, без текстуры)
                    LEATHER = 147, LEATHER2 = 148,   // Р92: кожа обмотки (палитра)
                    FLINT = 149, FLINT2 = 150,       // Р92: кремень головки топора (палитра)
                    HANDLE = 151, HANDLE2 = 152,     // Р92: светлое дерево рукоятей (палитра)
                    HIGHLIGHT = 153,                 // Р94: контур вырубки (палитра, яркий)
                    GRAVEL = 154,                    // Р101: щебень - выветрелая порода под почвой
                    // Р106: виды деревьев и кустов - своя кора (рисунок брёвен TFC) и своя листва (палитра)
                    BIRCH_BARK = 155, BIRCH_BARK2 = 156, ASPEN_BARK = 157, SPRUCE_BARK = 158, PINE_BARK = 159,
                    WILLOW_BARK = 160, HAZEL_BARK = 161, LINDEN_BARK = 162, ALDER_BARK = 163,
                    LEAF_BIRCH = 164, LEAF_BIRCH2 = 165, NEEDLE = 166, NEEDLE2 = 167, LEAF_WILLOW = 168, LEAF_WILLOW2 = 169,
                    LEAF_LIGHT = 170, LEAF_LIGHT2 = 171, BERRY = 172,
                    // Р107: травы и цветы (палитра), сено
                    GRASS_BLADE = 173, GRASS_BLADE2 = 174, FLOWER_Y = 175, FLOWER_B = 176, FLOWER_R = 177, FLOWER_W = 178,
                    CLOVER = 179, NETTLE = 180, FERN = 181, MOSS = 182, REED = 183, CATTAIL = 184, HAY = 185,
                    MATERIAL_COUNT = 186,
                    // Р89: снег в сетке - верхние 16 номеров 11-битного поля, в номере его толщина
                    SNOW_MESH = 2032 };
inline bool isGrass(Material m) { return m == GRASS || (m >= GRASS_FIRST && m <= GRASS_LAST); }
/** Номер почвы под дёрном (GRASS без почвы - суглинок). */
inline int grassSoil(Material m) { return m == GRASS ? 2 : int(m) - GRASS_FIRST; }

// Р106: кора и листва любого вида - для рубки, окорки, листопада
inline bool isBark(Material m) { return m == BARK || m == BARK2 || (m >= BIRCH_BARK && m <= ALDER_BARK); }
inline bool isLeaf(Material m) { return m == LEAVES || m == LEAVES2 || (m >= LEAF_BIRCH && m <= BERRY); }

struct ChunkPos { int32_t x, y, z; };

struct Chunk {
    ChunkPos pos{};
    bool edited = false;   // правился (Р2): маски сетки - из вокселей, а не формулой от высоты
    // порядок: x быстрее всех, потом z, потом y - срез по высоте (слой y) лежит в памяти подряд
    std::array<Material, CHUNK * CHUNK * CHUNK> voxels{};
    Material& at(int x, int y, int z) { return voxels[(y * CHUNK + z) * CHUNK + x]; }
    Material at(int x, int y, int z) const { return voxels[(y * CHUNK + z) * CHUNK + x]; }
};

}  // namespace world
