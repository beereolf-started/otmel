#pragma once
// Р105: ТЕЛО ИГРОКА - воксельный человек 1.8 м из шести частей: голова, туловище, две руки, две ноги
// (воксель 1/32 блока). Каждая часть - своя модель и свой сустав; поза - поворот частей в суставах:
// при ходьбе ноги и руки качаются в такт шагу, правая рука замахивается с инструментом, присед
// опускает тело, лёжа тело ложится вперёд. Видно из-за плеча (F4).
#include <memory>
#include <vector>

#include <glm/gtc/quaternion.hpp>

#include "game/items.h"

namespace game {

enum class BodyPartId : uint8_t { HEAD, TORSO, ARM_L, ARM_R, LEG_L, LEG_R, COUNT };

struct BodyPart {
    std::shared_ptr<ItemModel> model;
    glm::vec3 joint{0};    // сустав в вокселях модели
    glm::vec3 attach{0};   // где сустав у тела в покое: блоки от ступней, оси тела (x вправо, y вверх, z вперёд)
};
const std::vector<BodyPart>& bodyParts();

struct BodyPose {
    glm::dvec3 feet{0};    // блоки
    float yaw = 0;         // куда смотрит (как у камеры)
    float walkPhase = 0, walkAmp = 0;
    int posture = 0;       // 0 стоя, 1 присев, 2 лёжа
    float chopR = 0, chopL = 0;   // замах руки, радианы (плюс - вверх-назад)
};
struct PartXf { glm::dvec3 pos; glm::quat rot; };   // центр масс части в мире, поворот
/** Положение каждой части в мире; hand - точки кистей (левая, правая) и поворот рук для предмета. */
void bodyPose(const BodyPose& p, std::vector<PartXf>& out, glm::dvec3 handPos[2], glm::quat handRot[2]);

}  // namespace game
