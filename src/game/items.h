#pragma once
// Р92: ПРЕДМЕТЫ И РУКИ.
//
// Предмет - процедурная модель из мелких вокселей (1/16 блока): инструмент, сук, бревно, кора, коряга.
// Модель хранит сами воксели, а не «вид предмета»: срубленный сук - это ровно те воксели, что были у
// дерева, со следами зарубок. Её можно делить дальше и перекрашивать (окорка).
//
// В мире предмет - твёрдое тело: положение, поворот, скорость, вращение. Тяжесть, удар о блоки с
// трением: сук на склоне катится. Успокоившись, тело засыпает и не считается. Положенный клавишей V
// предмет стоит неподвижно, привязанный к вокселю (сам он может быть больше вокселя).
//
// Руки - два слота (Р90): ЛКМ берёт в левую, ПКМ в правую, если рука свободна.
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "world/chunk.h"
#include "world/query.h"
#include "game/rigid.h"

namespace game {

enum class ItemKind : uint8_t { AXE, SAW, DRAWKNIFE, SHOVEL, LOG, LIMB, TWIG, BARK, SNAG, STUMP };
enum class ItemSize : uint8_t { LARGE, MEDIUM, SMALL };

struct ItemModel {
    int sx = 0, sy = 0, sz = 0;
    int res = 16;                     // вокселей на блок: 16 у дерева и брёвен, 64 у инструментов (мелкие детали)
    glm::vec3 grip{0};                // где держит рука (воксели модели) - у инструментов
    std::vector<world::Material> v;   // [(y·sz + z)·sx + x], AIR - пусто
    glm::ivec3 center{0};             // центр масс, воксели модели (округлённо) - вокруг него вращается тело
    float mass = 0;                   // число вокселей
    glm::vec3 inertia{1};             // главные моменты (оси модели) относительно центра: воксели · блок²
    glm::vec3 lo{0}, hi{0};           // рамка относительно центра, блоки
    std::vector<glm::vec3> probes;    // точки касания на поверхности относительно центра, блоки
    world::Material at(int x, int y, int z) const { return v[(size_t(y) * sz + z) * sx + x]; }
    world::Material& at(int x, int y, int z) { return v[(size_t(y) * sz + z) * sx + x]; }
    /** После правки вокселей: центр, масса, инерция, рамка, точки касания. */
    void finish();
};

struct Item {
    ItemKind kind = ItemKind::AXE;
    ItemSize size = ItemSize::SMALL;
    std::shared_ptr<const ItemModel> model;
    bool tool() const { return kind <= ItemKind::SHOVEL; }
};
std::string itemName(const Item& it);
Item makeTool(ItemKind kind);

// Сетка модели: куски 32³ мелких вокселей; off - угол куска от центра масс, мелкие воксели
struct ItemMeshChunk { std::vector<uint32_t> verts; glm::ivec3 off; };
void meshItem(const ItemModel& m, std::vector<ItemMeshChunk>& out);

struct WorldItem {
    uint32_t id = 0;
    Item item;
    Body body;
    bool placed = false;        // положен клавишей V: неподвижен
    std::vector<uint32_t> meshes;   // номера сеток (render::MeshId), по кускам модели
    std::vector<glm::ivec3> offs;
};

class Items {
public:
    std::vector<WorldItem> list;
    uint32_t spawn(const Item& it, glm::dvec3 pos, glm::quat rot, glm::vec3 vel = glm::vec3(0), glm::vec3 ang = glm::vec3(0),
                   bool placed = false);
    // шаг тел у игрока (дальше радиуса не считаем), с подшагами
    void step(world::VoxelQuery& q, float dt, glm::dvec3 near, double radius);
    // какой предмет на луче (from, dir - блоки), ближе maxDist; -1 - никакой
    int pick(glm::dvec3 from, glm::dvec3 dir, double maxDist, double& tHit) const;
    // разбудить тела рядом (блок сломан, предмет убран)
    void wake(glm::dvec3 at, double radius);
private:
    uint32_t next_ = 1;
};

}  // namespace game
