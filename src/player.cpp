#include "player.h"

#include <algorithm>
#include <cmath>

namespace game {
namespace {

constexpr float UNIT = 16.0f;        // блоков в прежней мере (Р54)
constexpr float HALF_W = 0.3f;       // полширины тела, блоки
constexpr float HEIGHT = 1.8f;       // рост
constexpr float EYE = 1.62f;         // глаза от ног
constexpr float WALK = 4.3f;         // блоков в секунду - как привычный шаг
constexpr float SPRINT = 6.8f;
constexpr float GRAVITY = 28.0f;     // блоков в секунду за секунду
constexpr float JUMP = 8.4f;         // прыжок примерно на 1.25 блока
constexpr float DOUBLE_JUMP = 0.35f; // за сколько секунд второй прыжок считается двойным
constexpr float MAX_FALL = 60.0f;

int32_t ifloor(double v) { return int32_t(std::floor(v)); }

// Есть ли твёрдый блок в коробке тела, если ноги в точке (x, y, z).
//
// Координаты ОБЯЗАТЕЛЬНО двойной точности: мир простирается на сотни тысяч блоков, а у float там шаг
// уже в десятые доли блока - тело то проваливалось бы, то застревало в воздухе.
bool blocked(world::VoxelQuery& q, const glm::dvec3& f) {
    const int32_t x0 = ifloor(f.x - HALF_W), x1 = ifloor(f.x + HALF_W);
    const int32_t z0 = ifloor(f.z - HALF_W), z1 = ifloor(f.z + HALF_W);
    const int32_t y0 = ifloor(f.y), y1 = ifloor(f.y + HEIGHT - 1e-4);
    for (int32_t vy = y0; vy <= y1; ++vy)
        for (int32_t vz = z0; vz <= z1; ++vz)
            for (int32_t vx = x0; vx <= x1; ++vx)
                if (q.solid(vx, vy, vz)) return true;
    return false;
}

// то же, но только рельеф - без стволов
bool blockedTerrain(world::VoxelQuery& q, const glm::dvec3& f) {
    const int32_t x0 = ifloor(f.x - HALF_W), x1 = ifloor(f.x + HALF_W);
    const int32_t z0 = ifloor(f.z - HALF_W), z1 = ifloor(f.z + HALF_W);
    const int32_t y0 = ifloor(f.y), y1 = ifloor(f.y + HEIGHT - 1e-4);
    for (int32_t vy = y0; vy <= y1; ++vy)
        for (int32_t vz = z0; vz <= z1; ++vz)
            for (int32_t vx = x0; vx <= x1; ++vx)
                if (q.solidTerrain(vx, vy, vz)) return true;
    return false;
}

// Движение по одной оси с упором: малыми шагами, на упоре возвращаемся и гасим скорость по этой оси.
// Р94: упёрся в ствол (не в рельеф) по горизонтали - взбираемся на него, если хватает блока: так влезают
// на лежащее бревно, не подпрыгивая на каждом шагу.
bool slide(world::VoxelQuery& q, glm::dvec3& feet, double delta, int axis) {
    if (delta == 0.0) return false;
    const double dir = delta > 0 ? 1.0 : -1.0;
    double left = std::abs(delta);
    while (left > 1e-9) {
        const double d = std::min(left, 0.05) * dir;   // шаг мельче блока - упор находится точно
        left -= std::abs(d);
        const double was = feet[axis];
        feet[axis] += d;
        if (blocked(q, feet)) {
            if (axis != 1 && !blockedTerrain(q, feet)) {
                glm::dvec3 up = feet;
                bool climbed = false;
                for (double h = 0.25; h <= 1.05 && !climbed; h += 0.25) {
                    up.y = feet.y + h;
                    if (!blocked(q, up)) { feet = up; climbed = true; }
                }
                if (climbed) continue;
            }
            feet[axis] = was;
            return true;
        }
    }
    return false;
}

}  // namespace

void step(Player& pl, world::VoxelQuery& q, const Input& in, float yaw, float dt, float time) {
    dt = std::min(dt, 0.1f);   // на долгом кадре телом сквозь стену не проходим
    const glm::vec3 fwd(std::cos(yaw), 0.0f, std::sin(yaw));
    const glm::vec3 right(-fwd.z, 0.0f, fwd.x);
    glm::vec3 wish = fwd * in.forward + right * in.strafe;
    if (glm::length(wish) > 1.0f) wish = glm::normalize(wish);

    // Двойной прыжок переключает полёт в обе стороны
    if (in.jumpPressed) {
        if (time - pl.lastJump < DOUBLE_JUMP) {
            pl.flying = !pl.flying;
            pl.vel = glm::vec3(0);
            pl.lastJump = -10.0f;
        } else {
            pl.lastJump = time;
            if (!pl.flying && pl.onGround) pl.vel.y = JUMP;
        }
    }

    if (pl.flying) {   // полёт - как было: прямое движение без притяжения и упоров
        const float sp = (in.sprint ? 48.0f : 5.0f) * pl.flySpeed;
        glm::vec3 d = wish * sp * dt;
        d.y += (float(in.jumpHeld) - float(in.down)) * sp * dt;
        pl.eye += glm::dvec3(d) / double(UNIT);
        pl.onGround = false;
        return;
    }

    const float want = in.sprint ? SPRINT : WALK;
    pl.vel.x = wish.x * want;
    pl.vel.z = wish.z * want;
    pl.vel.y = std::max(-MAX_FALL, pl.vel.y - GRAVITY * dt);

    glm::dvec3 feet(pl.eye.x * UNIT, pl.eye.y * UNIT - EYE, pl.eye.z * UNIT);
    const glm::dvec3 feet0 = feet;
    if (slide(q, feet, double(pl.vel.y) * dt, 1)) pl.vel.y = 0.0f;
    slide(q, feet, double(pl.vel.x) * dt, 0);
    slide(q, feet, double(pl.vel.z) * dt, 2);
    // на земле - если прямо под ногами твёрдо; по этому же признаку разрешается прыжок
    glm::dvec3 probe = feet;
    probe.y -= 0.02;
    pl.onGround = blocked(q, probe);
    if (pl.onGround && pl.vel.y < 0.0f) pl.vel.y = 0.0f;
    pl.eye += (feet - feet0) / double(UNIT);
}

}  // namespace game
