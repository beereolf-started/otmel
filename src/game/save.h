#pragma once
// Р98: СОХРАНЕНИЕ МИРА.
//
// Мир - чистая функция зерна (Р2), поэтому хранится не мир, а то, чем игрок его изменил:
//   - правки блоков (сломанные блоки);
//   - зарубки, пни, перерубленные корни, выкопанные пни - по месту дерева в лесу;
//   - поваленные стволы целиком: их скелет (смятый, с отрубленными сучьями), кусок ствола, окорка, поза;
//   - предметы на земле - модель (воксели), поза, положен ли;
//   - игрок: где стоит, куда смотрит, что в руках.
// Один двоичный файл; пишется во временный и подменяет прежний - оборванная запись не портит сохранение.
// То же самое, что пишется в файл, потом пойдёт по сети (мультиплеер).
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "game/items.h"
#include "game/tree_body.h"
#include "world/query.h"

namespace game {

struct SaveData {
    uint32_t seed = 0;
    glm::dvec3 eye{0};
    float yaw = 0, pitch = 0;
    bool flying = false;
    std::vector<world::Edit> edits;
    std::optional<Item> hands[2];
    std::vector<std::pair<uint64_t, TreeCut>> cuts;
    std::vector<TreeBody> bodies;
    std::vector<WorldItem> items;
    std::vector<uint64_t> taken;   // Р99: подобранные природные камни и сушняк
};

bool saveWorld(const std::string& path, const SaveData& d);
bool loadWorld(const std::string& path, SaveData& d);

}  // namespace game
