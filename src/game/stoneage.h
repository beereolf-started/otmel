#pragma once
// Р99: КАМЕННЫЙ ВЕК - первая петля выживания.
//
// Подобрал камень и сушняк с земли -> выточил из камня головку (окно тёски, как в TFC) -> соскоблил
// камнем волокно коры с молодого дерева -> на земле в режиме крафта (F) сложил головку, палку и
// волокно -> получил орудие -> срубил дерево.
//
// Тёска - камень поднят к глазам сеткой 5×5; ЛКМ скалывает клетку под курсором. Как только оставшаяся
// форма совпала с образцом (или с его зеркалом) - камень становится головкой. Образцы - как в TFC.
// Крафт - сетка 2×2 ячеек по четверть блока на земле; рецепты без порядка: что лежит в ячейках.
#include <memory>
#include <string>
#include <vector>

#include "game/items.h"

namespace game {

Item makeStone(world::Material rock, uint32_t seed);   // камушек 5-9 см, неправильной формы
Item makeStick(uint32_t seed);                         // сухая ветка 0.6-1 м, с изломами
Item makeFiber();                                      // пучок волокна
Item makeClay();                                       // ком глины

struct Knap {
    int hand = 1;
    bool cell[5][5];
    int cx = 2, cy = 2;            // курсор
    float mx = 0, my = 0;          // накопленное движение мыши
    bool down = false;             // ЛКМ зажата - скалываем всё, над чем проходит курсор
    world::Material rock = world::AIR;
    Knap() { for (auto& r : cell) for (bool& c : r) c = true; }
};
/** Что вышло из камня: головка, если форма совпала с образцом; иначе STONE. */
ItemKind knapResult(const Knap& k);
Item makeHead(ItemKind kind, world::Material rock, const Knap& k);
/** Камень для тёски перед глазами: плита 5×5 клеток, курсор - светлой рамкой. */
std::shared_ptr<ItemModel> knapModel(const Knap& k);

/** Образцы тёски для панели: строка подписей и пять строк фигур (■ - камень остаётся, · - скалывается). */
std::vector<std::string> knapPatternsText();
/** Рецепт по предметам в ячейках крафта. Совпал - результат и его имя. */
bool matchRecipe(const std::vector<const Item*>& got, Item& out, std::string& name);
/** Подсказка: что из чего собирается (как EMI). */
std::string recipesHint();

}  // namespace game
