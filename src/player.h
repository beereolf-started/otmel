#pragma once
// Р83: ИГРОК НА ЗЕМЛЕ.
//
// До сих пор камера была свободной: летела куда угодно и ничего не задевала. Теперь у неё есть тело -
// коробка 0.6 × 1.8 блока, глаза на высоте 1.62, - притяжение и столкновения с блоками мира.
//
// Два состояния: пешком (притяжение, прыжок, бег) и полёт (как было, для осмотра). Переход - двойным
// прыжком в обе стороны.
//
// Всё внутри считается в БЛОКАХ и блоках в секунду, а наружу отдаётся в прежних мерах (16 блоков,
// Р54): положение камеры во всей остальной игре - в них.
#include <cstdint>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>

#include "world/query.h"

namespace game {

enum class Posture : uint8_t { STAND, CROUCH, PRONE };   // Р95: стоя, присев (Shift), лёжа (C)

struct Player {
    glm::dvec3 eye{0};        // глаза, прежние меры
    glm::vec3 vel{0};         // скорость, блоки в секунду
    bool flying = true;       // полёт: без притяжения и столкновений
    bool onGround = false;
    float lastJump = -10.0f;  // когда прыгнули прошлый раз (секунды от начала) - для двойного прыжка
    float flySpeed = 1.0f;    // множитель скорости полёта (колесо мыши)
    Posture posture = Posture::STAND;
    float eyeH = 1.62f;       // глаза от ног сейчас (плавно меняется с посадкой), блоки
};

// Ввод кадра: оси в плоскости земли, прыжок (нажатие, не удержание), бег, вверх-вниз в полёте.
struct Input {
    float forward = 0, strafe = 0;   // -1..1
    bool jumpPressed = false;        // именно нажатие в этом кадре
    bool jumpHeld = false;
    bool sprint = false;             // Р95: Ctrl - бег (в полёте - быстрее)
    bool crouch = false;             // Shift - присесть (в полёте - вниз)
    bool prone = false;              // C - лечь
};

// Шаг игрока: ввод, притяжение, столкновения. yaw - куда смотрит камера, time - секунды от начала.
void step(Player& pl, world::VoxelQuery& q, const Input& in, float yaw, float dt, float time);

}  // namespace game
