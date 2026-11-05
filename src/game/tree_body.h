#pragma once
// Р94: ПОВАЛЕННОЕ ДЕРЕВО - ТВЁРДОЕ ТЕЛО.
//
// Сначала дерево поворачивается вокруг недоруба (Р93). Недоруб ломается - на 30° или когда ствол
// чем-то коснулся земли - и дерево становится свободным телом: скорости берутся из вращения, дальше
// тяжесть, удары с трением, перекат и покой (game/rigid.h). Землю держат только жёсткие части - ствол
// и толстые сучья; тонкие ветви и крона сминаются (смятие - когда легло). Пень - тоже опора: со
// высокого пня ствол соскальзывает.
//
// Что упало, решает скелет, а не высота точки: ствол выше реза и всё, что от него растёт. Сучья ниже
// реза остаются у пня.
//
// Обрубка: луч по ветвям тела; сук под прицелом - контур у его основания; ударов - по толщине сука.
// Отрубленное (сук со всем, что на нём) - предмет своего класса: большой, средний, малый сук или ветка.
// Тело после этого легче и может перекатиться.
#include <memory>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>

#include "game/felling.h"
#include "game/items.h"
#include "game/rigid.h"

namespace game {

struct TreeBody {
    uint64_t key = 0;
    glm::ivec3 base{0}, shift{0};
    std::shared_ptr<world::TreeSkel> skel;   // своя копия: смятие, отрубленные ветви
    TreeCut cut;
    bool free = false;          // недоруб сломался
    Falling hinge;
    Body body;                  // центр масс в мире (блоки), поворот дерева
    float mass = 1;
    glm::vec3 inertia{1};
    std::vector<glm::vec3> probes;   // жёсткие точки, блоки от центра масс, в осях дерева
    glm::vec3 com{0};           // центр масс, мелкие воксели от основания
    bool crushed = false, meshDirty = true;
    std::vector<char> removed;  // ветвь отрублена
    int chopBranch = -1, chopHits = 0;

    glm::dvec3 origin() const { return glm::dvec3(base) * 16.0 + glm::dvec3(shift.x, -3, shift.z); }   // мелкие воксели
    glm::mat3 R() const { return glm::mat3_cast(body.rot); }
    glm::dvec3 toWorld(glm::vec3 local) const { return body.pos + glm::dvec3(R() * ((local - com) / 16.0f)); }   // блоки
    glm::vec3 toLocal(glm::dvec3 w) const { return glm::transpose(R()) * glm::vec3(w - body.pos) * 16.0f + com; }
    // ветвь b принадлежит телу (упала и не отрублена); для ствола - смотря по высоте
    bool owns(int b, float y) const {
        if (b < 0 || b >= int(skel->branches.size()) || skel->branches[size_t(b)].root) return false;
        if (b < int(removed.size()) && removed[size_t(b)]) return false;
        return cut.falls(b, y);
    }
};

TreeBody makeTreeBody(uint64_t key, glm::ivec3 base, glm::ivec3 shift, const world::TreeSkel& skel, const TreeCut& cut, glm::vec2 fallDir);
/** Масса, инерция и точки касания заново (после обрубки); положение тела в мире сохраняется. */
void reshape(TreeBody& tb);
/** Шаг: недоруб, затем свободное тело. solid(x, y, z) - рельеф и неподвижные стволы (пни), без самого тела. */
template <class Solid> void stepTreeBody(TreeBody& tb, Solid solid, float dt);
/** Легло: ветви, ушедшие в землю, сминаются по ней. */
template <class Solid> void crushTreeBody(TreeBody& tb, Solid solid);

/** Какая ветвь тела на луче (блоки); -1 - мимо. */
int hitBranch(const TreeBody& tb, glm::dvec3 from, glm::dvec3 dir, float maxBlocks, float& tBlocks);
/** Сколько ударов нужно, чтобы отрубить сук. */
int limbHits(const TreeBody& tb, int branch);
struct Severed { Item item; glm::dvec3 pos; glm::quat rot; glm::vec3 vel; };
/** Отрубить сук со всем, что на нём: предмет и его место в мире. */
bool severBranch(TreeBody& tb, int branch, Severed& out);
/** Контур вырубки у основания сука: модель-кольцо (воксели 1/16) и угол модели в осях дерева. */
std::shared_ptr<ItemModel> limbContour(const TreeBody& tb, int branch, glm::vec3& cornerLocal);

// ─── шаблоны
template <class Solid>
void stepTreeBody(TreeBody& tb, Solid solid, float dt) {
    if (!tb.free) {
        Falling& f = tb.hinge;
        constexpr float G = BODY_GRAVITY * 16.0f;
        const int sub = 8;
        const float h = std::min(dt, 0.05f) / sub;
        bool release = false;
        for (int s = 0; s < sub && !release; ++s) {
            const glm::mat3 Rh = glm::mat3_cast(f.rot());
            const glm::vec3 r = Rh * f.com;
            f.omega += glm::dot(glm::cross(r, glm::vec3(0, -G, 0)), f.axis) / f.I * h;
            f.theta += f.omega * h;
            if (f.theta < 0.01f) { f.theta = 0.01f; f.omega = std::max(f.omega, 0.05f); }
            const glm::mat3 R2 = glm::mat3_cast(f.rot());
            const glm::dvec3 hw = tb.origin() + glm::dvec3(f.hinge);
            for (const glm::vec3& p : f.probes) {
                const glm::dvec3 w = (hw + glm::dvec3(R2 * p)) / 16.0;
                if (solid(int32_t(std::floor(w.x)), int32_t(std::floor(w.y)), int32_t(std::floor(w.z)))) { release = true; break; }
            }
            if (f.theta > 0.52f) release = true;   // недоруб не держит дальше 30°
        }
        // поза тела из вращения вокруг шарнира
        const glm::mat3 Rh = glm::mat3_cast(f.rot());
        tb.body.rot = f.rot();
        tb.body.pos = (tb.origin() + glm::dvec3(f.hinge) + glm::dvec3(Rh * (tb.com - f.hinge))) / 16.0;
        if (release) {
            tb.free = true;
            const glm::vec3 w = f.axis * f.omega;
            tb.body.ang = w;
            tb.body.vel = glm::cross(w, Rh * (tb.com - f.hinge) / 16.0f);
        }
        return;
    }
    // дерево: трение сильное, отскока нет, качение гасится быстро, засыпает раньше мелочи
    stepRigid(tb.body, tb.mass, tb.inertia, tb.probes, solid, dt, 0.9f, 4.0f, 0.0f, 0.25f, 0.3f);
}

template <class Solid>
void crushTreeBody(TreeBody& tb, Solid solid) {
    world::TreeSkel& t = *tb.skel;
    const glm::mat3 Rt = glm::transpose(tb.R());
    std::vector<glm::vec3> delta(t.nodes.size(), glm::vec3(0));   // сдвиг в мире, блоки
    for (size_t i = 1; i < t.nodes.size(); ++i) {
        const world::TreeNode& n = t.nodes[i];
        if (!tb.owns(n.branch, n.p[1]) || n.r <= 0) continue;
        delta[i] = delta[size_t(n.parent)];
        const glm::dvec3 w = tb.toWorld(glm::vec3(n.p[0], n.p[1], n.p[2])) + glm::dvec3(delta[i]);
        const int32_t bx = int32_t(std::floor(w.x)), bz = int32_t(std::floor(w.z));
        int32_t by = int32_t(std::floor(w.y));
        if (!solid(bx, by, bz)) continue;
        for (int k = 0; k < 64 && solid(bx, by, bz); ++k) ++by;
        delta[i].y += std::max(0.0f, float(double(by) + n.r / 16.0 - w.y));
    }
    for (size_t i = 1; i < t.nodes.size(); ++i) {
        if (delta[i] == glm::vec3(0)) continue;
        const glm::vec3 d = Rt * delta[i] * 16.0f;
        for (int a = 0; a < 3; ++a) t.nodes[i].p[a] += d[a];
    }
    for (world::TreeLeaf& l : t.leaves) {
        const int last = t.branches[size_t(l.branch)].last;
        if (last < 0 || delta[size_t(last)] == glm::vec3(0)) continue;
        const glm::vec3 d = Rt * delta[size_t(last)] * 16.0f;
        for (int a = 0; a < 3; ++a) l.c[a] += d[a];
    }
    for (const world::TreeNode& n : t.nodes)
        for (int a = 0; a < 3; ++a) { t.lo[a] = std::min(t.lo[a], n.p[a] - n.r * 1.3f - 2); t.hi[a] = std::max(t.hi[a], n.p[a] + n.r * 1.3f + 2); }
    for (const world::TreeLeaf& l : t.leaves)
        for (int a = 0; a < 3; ++a) { t.lo[a] = std::min(t.lo[a], l.c[a] - l.r * 1.3f - 1); t.hi[a] = std::max(t.hi[a], l.c[a] + l.r * 1.3f + 1); }
    tb.crushed = true;
    tb.meshDirty = true;
}

}  // namespace game
