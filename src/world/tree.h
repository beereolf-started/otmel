#pragma once
// Р91: ДЕРЕВО ВЫРАЩИВАЕТСЯ, А НЕ СТРОИТСЯ ПО ПРАВИЛАМ.
//
// Прежний дуб (Р53-Р72) собирался правилами - ствол, развилка, отводы по золотому углу, - и выходил
// «парковым»: всё слишком правильное. Теперь крона - облако точек неправильной формы (перекошенное,
// с вмятинами), и побеги растут к ещё не занятым точкам (колонизация пространства, Runions 2007):
// где свет есть - туда и растёт, где нет - ветви не будет. Сверх того:
//   - ДУБОВЫЙ ЗИГЗАГ: на каждом шаге побег сворачивает то влево, то вправо - главная примета дуба,
//     особенно зимой без листвы;
//   - провисание: длинный пологий сук клонится под своим весом, концы сучьев смотрят вниз;
//   - толщина - трубочная модель (r^2.5 родителя = сумма r^2.5 детей), подогнанная под толщину комля;
//   - очищение снизу: нижние затенённые сучья отмерли - остались сухие обломки;
//   - ствол с наклоном и изгибом, развилка низкая.
//
// ДЕРЕВО - ГРАФ ВЕТВЕЙ (Р90). Ветвь - цепочка узлов от развилки до кончика; продолжение ветви - самый
// толстый ребёнок, остальные начинают новые ветви. У каждого вокселя есть номер ветви: игра знает, где
// ствол, где сук, где ветка, и где их стык - место будущей вырубки.
//
// Координаты - мелкие воксели (1/16 блока) от основания ствола, y вверх.
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "world/chunk.h"

namespace world {

enum class TreeAge : uint8_t { YOUNG, ADULT, GIANT };
// Р106: виды. Деревья - из того же генератора со своими пропорциями, ветвлением, корой и листвой; кусты -
// несколько стволов от земли без общего ствола.
enum class TreeSpecies : uint8_t { OAK, BIRCH, ASPEN, LINDEN, ALDER, SPRUCE, PINE, WILLOW_SHRUB, HAZEL, RASPBERRY, ROSEHIP, COUNT };
const char* speciesName(TreeSpecies s);
inline bool isShrub(TreeSpecies s) { return s >= TreeSpecies::WILLOW_SHRUB; }

struct TreeNode {
    float p[3];
    float r;            // радиус, мелкие воксели
    int parent;         // -1 у комля
    int branch;         // номер ветви
};
struct TreeBranch {
    int parent;         // ветвь, от которой отходит (-1 у ствола и корней)
    int first, last;    // первый и последний узел (first - узел развилки на родителе НЕ входит)
    int order;          // 0 - ствол, 1 - сук от ствола, ...
    bool root, dead;    // корень; сухой обломок
    float baseR;        // толщина у основания, мелкие воксели
};
struct TreeLeaf { float c[3]; float r; int branch; };

struct TreeSkel {
    TreeAge age = TreeAge::YOUNG;
    TreeSpecies species = TreeSpecies::OAK;
    uint32_t seed = 0;
    std::vector<TreeNode> nodes;
    std::vector<TreeBranch> branches;
    std::vector<TreeLeaf> leaves;
    float lo[3] = {0, 0, 0}, hi[3] = {0, 0, 0};   // рамка дерева с листвой, мелкие воксели
    float height = 0;      // верх кроны, мелкие воксели
    float crownR = 0;      // радиус кроны по горизонтали, мелкие воксели (для теней и спрайтов)
    int roots = 0;         // корней от комля (Р90: по ним падают коряги)
};

/** Вырастить дерево. Одно семя и возраст - всегда одно и то же дерево. */
std::shared_ptr<const TreeSkel> growTree(uint32_t seed, TreeAge age, TreeSpecies species = TreeSpecies::OAK);

/**
 * Воксели куска 32³ в вокселях размера scale (мелких): кусок (cx, cy, cz) покрывает [c·32, c·32 + 32)
 * от основания ствола. leaves - рисовать ли листву (времена года). branchOut - номер ветви каждого
 * вокселя (или nullptr). Ответ - есть ли в куске хоть что-то.
 */
// Раскладка отрезков и листвы по кускам 32³ вокселей размера scale: кусок перебирает только своё.
struct TreeIndex {
    struct Cell { std::vector<int> segs, leaves; };
    int scale = 1;
    std::unordered_map<uint64_t, Cell> cells;
    static uint64_t key(int x, int y, int z) {
        return (uint64_t(uint32_t(x + 4096)) << 42) ^ (uint64_t(uint32_t(y + 4096)) << 21) ^ uint64_t(uint32_t(z + 4096));
    }
};
TreeIndex treeIndex(const TreeSkel& t, int scale);
bool treeChunk(const TreeSkel& t, const TreeIndex& ix, int cx, int cy, int cz, bool wood, bool leaves, Material* out,
               uint16_t* branchOut = nullptr);
/** Какие куски вообще задевает дерево (включительно), в кусках 32³ вокселей размера scale. */
void treeChunkRange(const TreeSkel& t, int scale, int lo[3], int hi[3]);
/** Мельче этого размера вокселя листву не рисуем: у великана она иначе уходит в десятки миллионов. */
int treeLeafScale(TreeAge age);

}  // namespace world
