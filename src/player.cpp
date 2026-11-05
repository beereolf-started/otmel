#include "player.h"

#include <algorithm>
#include <cmath>

namespace game {
namespace {

constexpr float UNIT = 16.0f;        // блоков в прежней мере (Р54)
constexpr float HALF_W = 0.3f;       // полширины тела, блоки
constexpr float GRAVITY = 28.0f;     // блоков в секунду за секунду
constexpr float JUMP = 8.4f;         // прыжок примерно на 1.25 блока
constexpr float DOUBLE_JUMP = 0.35f; // за сколько секунд второй прыжок считается двойным
constexpr float MAX_FALL = 60.0f;
// Р95: стоя, присев, лёжа - рост, глаза, скорость (блоки, блоки/с)
struct PostureDef { float height, eye, walk, sprint; };
constexpr PostureDef POSTURES[3] = {{1.8f, 1.62f, 4.3f, 6.8f}, {1.5f, 1.27f, 2.0f, 2.0f}, {0.6f, 0.42f, 1.0f, 1.0f}};

int32_t ifloor(double v) { return int32_t(std::floor(v)); }

// Есть ли твёрдый блок в коробке тела высотой h, если ноги в точке f. terrain - только рельеф, без стволов.
//
// Координаты ОБЯЗАТЕЛЬНО двойной точности: мир простирается на сотни тысяч блоков, а у float там шаг
// уже в десятые доли блока - тело то проваливалось бы, то застревало в воздухе.
bool blocked(world::VoxelQuery& q, const glm::dvec3& f, float h, bool terrain = false) {
    const int32_t x0 = ifloor(f.x - HALF_W), x1 = ifloor(f.x + HALF_W);
    const int32_t z0 = ifloor(f.z - HALF_W), z1 = ifloor(f.z + HALF_W);
    const int32_t y0 = ifloor(f.y), y1 = ifloor(f.y + h - 1e-4);
    for (int32_t vy = y0; vy <= y1; ++vy)
        for (int32_t vz = z0; vz <= z1; ++vz)
            for (int32_t vx = x0; vx <= x1; ++vx)
                if (terrain ? q.solidTerrain(vx, vy, vz) : q.solid(vx, vy, vz)) return true;
    return false;
}

// Движение по одной оси с упором: малыми шагами, на упоре возвращаемся и гасим скорость по этой оси.
// Р94: упёрся в ствол (не в рельеф) по горизонтали - взбираемся на него, если хватает блока: так влезают
// на лежащее бревно, не подпрыгивая на каждом шагу.
bool slide(world::VoxelQuery& q, glm::dvec3& feet, double delta, int axis, float h) {
    if (delta == 0.0) return false;
    const double dir = delta > 0 ? 1.0 : -1.0;
    double left = std::abs(delta);
    while (left > 1e-9) {
        const double d = std::min(left, 0.05) * dir;   // шаг мельче блока - упор находится точно
        left -= std::abs(d);
        const double was = feet[axis];
        feet[axis] += d;
        if (blocked(q, feet, h)) {
            if (axis != 1 && !blocked(q, feet, h, true)) {
                glm::dvec3 up = feet;
                bool climbed = false;
                for (double s = 0.25; s <= 1.05 && !climbed; s += 0.25) {
                    up.y = feet.y + s;
                    if (!blocked(q, up, h)) { feet = up; climbed = true; }
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
            if (!pl.flying && pl.onGround && pl.posture == Posture::STAND) pl.vel.y = JUMP;
        }
    }

    if (pl.flying) {   // полёт: Ctrl - быстрее, пробел - вверх, Shift - вниз
        const float sp = (in.sprint ? 48.0f : 5.0f) * pl.flySpeed;
        glm::vec3 d = wish * sp * dt;
        d.y += (float(in.jumpHeld) - float(in.crouch)) * sp * dt;
        pl.eye += glm::dvec3(d) / double(UNIT);
        pl.onGround = false;
        pl.posture = Posture::STAND;
        pl.eyeH = POSTURES[0].eye;
        return;
    }

    glm::dvec3 feet(pl.eye.x * UNIT, pl.eye.y * UNIT - pl.eyeH, pl.eye.z * UNIT);
    // Р95: посадка. Встать можно, только если над головой хватает места.
    Posture want = in.prone ? Posture::PRONE : (in.crouch ? Posture::CROUCH : Posture::STAND);
    if (int(want) < int(pl.posture) && blocked(q, feet, POSTURES[int(want)].height))
        want = (want == Posture::STAND && !blocked(q, feet, POSTURES[1].height)) ? Posture::CROUCH : pl.posture;
    pl.posture = want;
    const PostureDef& P = POSTURES[int(pl.posture)];
    const float H = P.height;

    // Р95: ВЫДАВЛИВАНИЕ. Тело оказалось внутри ствола (дерево упало на игрока) - в ближайшее свободное место:
    // вбок и вверх, до трёх блоков. Иначе выбраться можно было только полётом.
    if (blocked(q, feet, H)) {
        double best = 1e9;
        glm::dvec3 to = feet;
        for (int dy = 0; dy <= 12; ++dy)
            for (int dz = -8; dz <= 8; ++dz)
                for (int dx = -8; dx <= 8; ++dx) {
                    const double cost = std::sqrt(double(dx * dx + dz * dz)) * 0.25 + dy * 0.25 * 0.7;
                    if (cost >= best) continue;
                    const glm::dvec3 c = feet + glm::dvec3(dx * 0.25, dy * 0.25, dz * 0.25);
                    if (blocked(q, c, H)) continue;
                    best = cost;
                    to = c;
                }
        feet = to;
        pl.vel.y = std::max(pl.vel.y, 0.0f);
    }

    const float speed = in.sprint && pl.posture == Posture::STAND ? P.sprint : P.walk;
    pl.vel.x = wish.x * speed;
    pl.vel.z = wish.z * speed;
    pl.vel.y = std::max(-MAX_FALL, pl.vel.y - GRAVITY * dt);

    if (slide(q, feet, double(pl.vel.y) * dt, 1, H)) pl.vel.y = 0.0f;
    slide(q, feet, double(pl.vel.x) * dt, 0, H);
    slide(q, feet, double(pl.vel.z) * dt, 2, H);
    // на земле - если прямо под ногами твёрдо; по этому же признаку разрешается прыжок
    glm::dvec3 probe = feet;
    probe.y -= 0.02;
    pl.onGround = blocked(q, probe, H);
    if (pl.onGround && pl.vel.y < 0.0f) pl.vel.y = 0.0f;
    // глаза плавно опускаются и поднимаются со сменой посадки
    pl.eyeH += (P.eye - pl.eyeH) * std::min(1.0f, dt * 12.0f);
    pl.eye = glm::dvec3(feet.x, feet.y + pl.eyeH, feet.z) / double(UNIT);
}

}  // namespace game
