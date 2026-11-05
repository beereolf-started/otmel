#pragma once
// Р93: РУБКА И ПАДЕНИЕ ДЕРЕВА.
//
// Удар топором по стволу вырезает КЛИН - V-образную зарубку со стороны игрока; каждый удар углубляет
// её. Число ударов - по толщине: молодому ~4, взрослому ~8, вековому ~16. Когда целого сечения
// остаётся меньше четверти, недоруб становится шарниром и дерево валится - от игрока.
//
// Падение - вращение твёрдого тела вокруг шарника под тяжестью: θ'' = τ / I, где τ - момент силы
// тяжести центра масс, I - момент инерции дерева относительно оси шарнира. Масса и инерция - по самим
// отрезкам скелета (объём r²·длина): кривое дерево падает криво. Касание земли - любым узлом кроны или
// ствола: отскок и покой. Ниже реза остаётся пень с корнями.
//
// Координаты дерева - мелкие воксели от основания ствола (как у world::TreeSkel).
#include <memory>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include "world/query.h"
#include "world/tree.h"

namespace game {

struct TreeCut {
    bool notched = false, felled = false;
    float y0 = 0;            // высота реза, мелкие воксели
    glm::vec2 dir{1, 0};     // откуда рубят: от оси ствола к игроку (по горизонтали)
    glm::vec2 center{0};     // ось ствола на высоте реза
    float R = 1;             // радиус ствола на высоте реза
    float depth = 0;         // глубина зарубки от поверхности
    int hits = 0;
    // воксель (мелкие воксели от основания) вырублен зарубкой
    bool inNotch(float x, float y, float z) const;
    // сколько ударов нужно - по толщине ствола
    float perHit() const;
    bool ready() const { return depth >= 2.0f * R * 0.75f; }
    // Р94: какие ветви падают вместе со стволом - по скелету: сук, растущий от ствола выше реза, и всё на нём
    std::vector<char> part;
    void computeParts(const world::TreeSkel& t);
    // Р96: пень - перерубленные корни (бит на ветвь-корень по порядку корней), выкопан ли
    uint32_t rootsCut = 0;
    std::vector<int> rootOf;   // ветвь -> номер корня от комля, к которому она принадлежит (-1 - не корень)
    int roots = 0;             // корней от комля
    bool allRootsCut() const { return roots > 0 && rootsCut == (roots >= 32 ? 0xFFFFFFFFu : ((1u << roots) - 1u)); }
    int rootTarget = -1, rootHits = 0;
    bool dug = false;
    int digHits = 0;
    // Р102: сколько листвы осталось (от ударов осыпается до четверти) и сколько коры содрано с живого дерева
    float leafKeep = 1.0f;
    int scraped = 0;
    // ветвь b сохраняет листву (жребий ветви против доли оставшегося)
    bool leafStays(int b) const { return float((uint32_t(b) * 2654435761u >> 12) & 1023u) / 1023.0f <= leafKeep; }
    bool falls(int branch, float y) const {
        if (branch == 0) return y >= y0;
        return branch > 0 && branch < int(part.size()) && part[size_t(branch)];
    }
};

/** Ось и радиус ствола на высоте y (по отрезкам ветви 0). */
bool trunkAt(const world::TreeSkel& t, float y, glm::vec2& center, float& r);
/** Луч (мелкие воксели от основания) по стволу: точка попадания. */
bool hitTrunk(const world::TreeSkel& t, glm::vec3 o, glm::vec3 d, float maxT, float& tHit, glm::vec3& p, float maxY = 1e9f);

struct Falling {
    glm::vec3 hinge{0};      // шарнир, мелкие воксели от основания
    glm::vec3 axis{1, 0, 0}; // ось вращения (горизонтальна)
    float theta = 0, omega = 0;
    float I = 1;             // момент инерции / масса, мелкие воксели²
    glm::vec3 com{0};        // центр масс от шарнира (до поворота), мелкие воксели
    std::vector<glm::vec3> probes;   // узлы для касания (от шарнира)
    bool done = false;
    float rest = 0;
    glm::quat rot() const { return glm::angleAxis(theta, axis); }
};
/** Начать падение: часть дерева выше реза, падает в сторону fallDir (горизонталь). */
Falling startFall(const world::TreeSkel& t, const TreeCut& cut, glm::vec2 fallDir);
/** Шаг падения. origin - мировое положение основания дерева в мелких вокселях. */
void stepFall(Falling& f, world::VoxelQuery& q, glm::dvec3 originFine, float dt);
/**
 * Р93: легло - ветви, ушедшие в землю, сминаются: узел под землёй поднимается на её поверхность, всё,
 * что растёт от него дальше, едет вместе с ним. Ответ - скелет с прижатыми к земле ветвями.
 */
std::shared_ptr<world::TreeSkel> crushFallen(const world::TreeSkel& t, const Falling& f, world::VoxelQuery& q, glm::dvec3 originFine,
                                             float y0);

}  // namespace game
