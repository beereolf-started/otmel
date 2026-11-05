#pragma once
// Р94: ТВЁРДОЕ ТЕЛО О БЛОКИ - общее для предметов и поваленных деревьев.
//
// Тело - масса, главные моменты инерции (в осях тела) и точки касания на поверхности (блоки от центра
// масс). Шаг: тяжесть; каждая точка внутри твёрдого блока выталкивается к ближайшей свободной грани
// (вверх охотнее), её скорость против нормали гасится импульсом с отскоком и трением Кулона. Подшаги
// по 1/240 с. Почти без движения полсекунды - тело засыпает.
#include <algorithm>
#include <cmath>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/mat3x3.hpp>

namespace game {

struct Body {
    glm::dvec3 pos{0};          // центр масс, блоки
    glm::quat rot{1, 0, 0, 0};
    glm::vec3 vel{0}, ang{0};   // блоки/с, рад/с
    bool asleep = false;
    float still = 0;            // сколько секунд почти без движения
};

constexpr float BODY_GRAVITY = 28.0f;   // как у игрока (player.cpp)

// solid(x, y, z) - твёрд ли блок
template <class Solid>
void stepRigid(Body& b, float mass, glm::vec3 inertia, const std::vector<glm::vec3>& probes, Solid solid, float dt,
               float friction = 0.6f, float rolling = 1.0f, float bounce = 0.15f, float sleepV = 0.08f, float sleepW = 0.15f) {
    if (b.asleep) return;
    dt = std::min(dt, 0.1f);
    const int sub = std::max(1, int(std::ceil(dt / (1.0f / 240.0f))));
    const float h = dt / float(sub);
    const glm::vec3 invI = 1.0f / inertia;
    const float invM = 1.0f / mass;
    for (int s = 0; s < sub; ++s) {
        b.vel.y -= BODY_GRAVITY * h;
        const glm::mat3 R = glm::mat3_cast(b.rot);
        const glm::mat3 Iw = R * glm::mat3(glm::vec3(invI.x, 0, 0), glm::vec3(0, invI.y, 0), glm::vec3(0, 0, invI.z)) * glm::transpose(R);
        int contacts = 0;
        glm::dvec3 push(0);
        for (const glm::vec3& pr : probes) {
            const glm::vec3 r = R * pr;
            const glm::dvec3 p = b.pos + glm::dvec3(r);
            const int32_t ix = int32_t(std::floor(p.x)), iy = int32_t(std::floor(p.y)), iz = int32_t(std::floor(p.z));
            if (!solid(ix, iy, iz)) continue;
            const double fx = p.x - ix, fy = p.y - iy, fz = p.z - iz;
            struct F { double d; glm::vec3 n; int dx, dy, dz; };
            const F faces[6] = {{1 - fy, {0, 1, 0}, 0, 1, 0}, {fx, {-1, 0, 0}, -1, 0, 0}, {1 - fx, {1, 0, 0}, 1, 0, 0},
                                {fz, {0, 0, -1}, 0, 0, -1}, {1 - fz, {0, 0, 1}, 0, 0, 1}, {fy, {0, -1, 0}, 0, -1, 0}};
            F best{1e9, {0, 1, 0}, 0, 1, 0};
            for (const F& f : faces) {
                if (solid(ix + f.dx, iy + f.dy, iz + f.dz)) continue;
                const double d = f.n.y > 0 ? f.d * 0.8 : f.d;   // вверх выталкиваем охотнее
                if (d < best.d) best = {d, f.n, f.dx, f.dy, f.dz};
            }
            if (best.d > 1e8) best = {1 - fy, {0, 1, 0}, 0, 1, 0};
            const glm::vec3 n = best.n;
            const float depth = float(best.n.y > 0 ? 1 - fy : best.d);
            ++contacts;
            push += glm::dvec3(n) * double(depth);
            const glm::vec3 vp = b.vel + glm::cross(b.ang, r);
            const float vn = glm::dot(vp, n);
            if (vn < 0) {
                const glm::vec3 rn = glm::cross(r, n);
                const float k = invM + glm::dot(n, glm::cross(Iw * rn, r));
                const float e = vn < -2.0f ? bounce : 0.0f;
                const float j = -(1 + e) * vn / k;
                glm::vec3 imp = n * j;
                glm::vec3 vt = vp - n * vn;
                const float vtl = glm::length(vt);
                if (vtl > 1e-5f) {
                    const glm::vec3 t = vt / vtl;
                    const glm::vec3 rt = glm::cross(r, t);
                    const float kt = invM + glm::dot(t, glm::cross(Iw * rt, r));
                    imp -= t * std::min(vtl / kt, friction * j);
                }
                b.vel += imp * invM;
                b.ang += Iw * glm::cross(r, imp);
            }
        }
        if (contacts) {
            b.pos += push * (0.5 / contacts);
            // тормоз качения: тело на опоре не раскачивается без конца, как на канате
            const float k = std::max(0.0f, 1.0f - rolling * h);
            b.ang *= k;
            b.vel = glm::vec3(b.vel.x * k, b.vel.y, b.vel.z * k);
        }
        b.ang *= 0.999f;
        b.pos += glm::dvec3(b.vel) * double(h);
        const float al = glm::length(b.ang);
        if (al > 1e-6f) b.rot = glm::normalize(glm::angleAxis(al * h, b.ang / al) * b.rot);
    }
    if (glm::length(b.vel) < sleepV && glm::length(b.ang) < sleepW) b.still += dt;
    else b.still = 0;
    if (b.still > 0.5f) { b.asleep = true; b.vel = glm::vec3(0); b.ang = glm::vec3(0); }
    if (b.pos.y < -4096) b.asleep = true;
}

}  // namespace game
