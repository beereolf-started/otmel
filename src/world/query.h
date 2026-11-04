#pragma once
// Р83: ЧТЕНИЕ МИРА ПО ОДНОМУ ВОКСЕЛЮ.
//
// Мир - чистая функция зерна, вокселей нигде не хранится. Но игроку нужно спрашивать про отдельные
// точки: во что он упёрся, что под прицелом. Считать ради одного вокселя целый кусок - дорого,
// поэтому здесь кусок считается один раз и держится до следующего вопроса о другом куске. Шаг по
// одному столбцу кусков (32 блока) обходится даром.
//
// Сюда же стекаются правки игрока (Р83): сломанный блок - это запись «здесь воздух», и читатель
// обязан её видеть, иначе игрок провалится сквозь то, чего уже нет.
#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "world/chunk.h"
#include "world/terrain.h"

namespace world {

// Правки мира: ключ - воксель (блок мира, Р54), значение - новый материал.
//
// Читают их потоки постройки, пишет поток кадра, поэтому снимок меняется ЦЕЛИКОМ: при правке
// собирается новая таблица и подменяется указатель. Правок немного, а гонок так нет вовсе.
// Координаты хранятся и в значении: ключ сжат и обратно не разбирается, а сетке нужно обойти
// правки своего куска.
struct Edit { int32_t x, y, z; Material m; };
using EditMap = std::unordered_map<uint64_t, Edit>;

inline uint64_t editKey(int32_t x, int32_t y, int32_t z) {
    return (uint64_t(uint32_t(x)) << 42) ^ (uint64_t(uint32_t(y) & 0x3FFFFF) << 21) ^ uint64_t(uint32_t(z) & 0x1FFFFF);
}

class Edits {
public:
    std::shared_ptr<const EditMap> snapshot() const {
        std::lock_guard lk(mu_);
        return map_;
    }
    void set(int32_t x, int32_t y, int32_t z, Material m) {
        std::lock_guard lk(mu_);
        auto next = std::make_shared<EditMap>(*map_);
        (*next)[editKey(x, y, z)] = Edit{x, y, z, m};
        map_ = std::move(next);
    }
    bool get(int32_t x, int32_t y, int32_t z, Material& out) const {
        const auto snap = snapshot();
        const auto it = snap->find(editKey(x, y, z));
        if (it == snap->end()) return false;
        out = it->second.m;
        return true;
    }
    size_t size() const { return snapshot()->size(); }

private:
    mutable std::mutex mu_;
    std::shared_ptr<const EditMap> map_ = std::make_shared<EditMap>();
};

// Материал вокселя мира с учётом пещер, среза земли (Р79), снежного слоя (Р82) и правок игрока.
class VoxelQuery {
public:
    explicit VoxelQuery(const TerrainParams& p, const Edits* edits = nullptr) : p_(p), edits_(edits) {}
    Material at(int32_t vx, int32_t vy, int32_t vz);
    // Можно ли сквозь воксель пройти: воздух и вода - да, остальное - нет
    bool solid(int32_t vx, int32_t vy, int32_t vz) {
        const Material m = at(vx, vy, vz);
        return m != AIR && m != WATER;
    }

private:
    const TerrainParams& p_;
    const Edits* edits_;
    std::shared_ptr<const EditMap> snap_;   // свой снимок правок - брать его на каждый воксель дорого
    size_t snapSize_ = 0;
    // Несколько столбцов кусков разом. Один не годится: тело игрока шириной меньше блока всё равно
    // стоит на границе двух столбцов, и на каждый воксель пересчитывался бы целый кусок 34×34 -
    // это миллисекунды на кадр вместо микросекунд.
    struct Slot {
        int32_t cx = INT32_MIN, cz = INT32_MIN, cy = INT32_MIN;
        bool caves = false;
        ColumnHeights hm{};
        uint64_t air[CHUNK + 2][CHUNK + 2]{};
        uint8_t cut[CHUNK + 2][CHUNK + 2]{};
    };
    static constexpr int SLOTS = 4;
    Slot slots_[SLOTS];
    int next_ = 0;
    Slot& slotFor(int32_t cx, int32_t cz);
};

}  // namespace world
