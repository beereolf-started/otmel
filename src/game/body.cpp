#include "game/body.h"

#include <cmath>

#include <glm/mat3x3.hpp>

namespace game {
namespace {

constexpr float RES = 32.0f;

std::shared_ptr<ItemModel> part(int sx, int sy, int sz) {
    auto m = std::make_shared<ItemModel>();
    m->res = 32;
    m->sx = sx; m->sy = sy; m->sz = sz;
    m->v.assign(size_t(sx) * sy * sz, world::AIR);
    return m;
}

std::vector<BodyPart> build() {
    using namespace world;
    std::vector<BodyPart> P(static_cast<size_t>(BodyPartId::COUNT));
    {   // голова: лицо - кожа, волосы сверху и сзади, глаза
        auto m = part(8, 8, 8);
        for (int y = 0; y < 8; ++y) for (int z = 0; z < 8; ++z) for (int x = 0; x < 8; ++x)
            m->at(x, y, z) = (y >= 6 || z <= 1 || (y >= 4 && (x == 0 || x == 7) && z <= 3)) ? BARK2 : HANDLE;
        m->at(2, 4, 7) = m->at(5, 4, 7) = BARK2;
        m->finish();
        P[size_t(BodyPartId::HEAD)] = {m, {4, 0, 4}, {0, 1.45f, 0}};
    }
    {   // туловище: кожаная рубаха, пояс
        auto m = part(13, 19, 7);
        for (int y = 0; y < 19; ++y) for (int z = 0; z < 7; ++z) for (int x = 0; x < 13; ++x)
            m->at(x, y, z) = y >= 2 && y <= 3 ? LEATHER2 : LEATHER;
        m->finish();
        P[size_t(BodyPartId::TORSO)] = {m, {6.5f, 0, 3.5f}, {0, 0.85f, 0}};
    }
    for (int s = 0; s < 2; ++s) {   // руки: рукав и кисть
        auto m = part(5, 22, 5);
        for (int y = 0; y < 22; ++y) for (int z = 0; z < 5; ++z) for (int x = 0; x < 5; ++x) m->at(x, y, z) = y < 5 ? HANDLE : LEATHER;
        m->finish();
        P[size_t(s ? BodyPartId::ARM_R : BodyPartId::ARM_L)] = {m, {2.5f, 21, 2.5f}, {s ? 0.28f : -0.28f, 1.40f, 0}};
    }
    for (int s = 0; s < 2; ++s) {   // ноги: штанина и сапог
        auto m = part(6, 27, 6);
        for (int y = 0; y < 27; ++y) for (int z = 0; z < 6; ++z) for (int x = 0; x < 6; ++x) m->at(x, y, z) = y < 4 ? BARK2 : LEATHER2;
        m->finish();
        P[size_t(s ? BodyPartId::LEG_R : BodyPartId::LEG_L)] = {m, {3, 27, 3}, {s ? 0.1f : -0.1f, 0.85f, 0}};
    }
    return P;
}

}  // namespace

const std::vector<BodyPart>& bodyParts() {
    static const std::vector<BodyPart> P = build();
    return P;
}

void bodyPose(const BodyPose& p, std::vector<PartXf>& out, glm::dvec3 handPos[2], glm::quat handRot[2]) {
    const auto& P = bodyParts();
    out.assign(P.size(), PartXf{});
    // поворот тела: ось z модели - туда, куда смотрит игрок
    glm::quat body = glm::angleAxis(1.5707963f - p.yaw, glm::vec3(0, 1, 0));
    glm::dvec3 root = p.feet;
    float torsoTilt = 0, legBend = 0;
    if (p.posture == 1) { root.y -= 0.3; torsoTilt = 0.35f; legBend = 0.9f; }    // присел
    if (p.posture == 2) {                                                          // лёг: тело вперёд, вдоль земли
        body = body * glm::angleAxis(1.5707963f, glm::vec3(1, 0, 0));
        root += glm::dvec3(0, 0.15, 0);
    }
    const float swing = std::sin(p.walkPhase) * 0.7f * p.walkAmp;
    for (size_t i = 0; i < P.size(); ++i) {
        const BodyPart& bp = P[i];
        glm::quat local(1, 0, 0, 0);
        glm::vec3 attach = bp.attach;
        switch (BodyPartId(i)) {
            case BodyPartId::LEG_L: local = glm::angleAxis(swing - legBend, glm::vec3(1, 0, 0)); break;
            case BodyPartId::LEG_R: local = glm::angleAxis(-swing - legBend, glm::vec3(1, 0, 0)); break;
            case BodyPartId::ARM_L: local = glm::angleAxis(-swing * 0.6f - p.chopL, glm::vec3(1, 0, 0)); break;
            case BodyPartId::ARM_R: local = glm::angleAxis(swing * 0.6f - p.chopR, glm::vec3(1, 0, 0)); break;
            default: break;
        }
        // туловище, голова и руки наклоняются вперёд при приседе - от пояса
        const glm::quat tilt = glm::angleAxis(torsoTilt, glm::vec3(1, 0, 0));
        const bool upper = BodyPartId(i) == BodyPartId::TORSO || BodyPartId(i) == BodyPartId::HEAD ||
                           BodyPartId(i) == BodyPartId::ARM_L || BodyPartId(i) == BodyPartId::ARM_R;
        if (upper) {
            const glm::vec3 waist(0, 0.85f, 0);
            attach = waist + glm::mat3_cast(tilt) * (attach - waist);
            local = tilt * local;
        }
        const glm::quat R = body * local;
        const glm::dvec3 joint = root + glm::dvec3(glm::mat3_cast(body) * attach);
        const glm::vec3 com = glm::vec3(bp.model->center) + 0.5f;
        out[i] = {joint + glm::dvec3(glm::mat3_cast(R) * ((com - bp.joint) / RES)), R};
        if (BodyPartId(i) == BodyPartId::ARM_L || BodyPartId(i) == BodyPartId::ARM_R) {
            const int h = BodyPartId(i) == BodyPartId::ARM_R ? 1 : 0;
            handPos[h] = joint + glm::dvec3(glm::mat3_cast(R) * glm::vec3(0, -0.62f, 0.04f));
            handRot[h] = R;
        }
    }
}

}  // namespace game
