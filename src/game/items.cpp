#include "game/items.h"

#include <algorithm>
#include <cmath>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/mat3x3.hpp>

#include "world/mesher.h"
#include "world/terrain.h"

namespace game {
namespace {


std::shared_ptr<ItemModel> blank(int sx, int sy, int sz) {
    auto m = std::make_shared<ItemModel>();
    m->sx = sx; m->sy = sy; m->sz = sz;
    m->v.assign(size_t(sx) * sy * sz, world::AIR);
    return m;
}
void box(ItemModel& m, int x0, int y0, int z0, int x1, int y1, int z1, world::Material mat) {
    for (int y = std::max(0, y0); y <= std::min(m.sy - 1, y1); ++y)
        for (int z = std::max(0, z0); z <= std::min(m.sz - 1, z1); ++z)
            for (int x = std::max(0, x0); x <= std::min(m.sx - 1, x1); ++x) m.at(x, y, z) = mat;
}

}  // namespace

void ItemModel::finish() {
    const float FINE = float(res);
    glm::dvec3 s(0);
    double n = 0;
    for (int y = 0; y < sy; ++y)
        for (int z = 0; z < sz; ++z)
            for (int x = 0; x < sx; ++x)
                if (at(x, y, z) != world::AIR) { s += glm::dvec3(x + 0.5, y + 0.5, z + 0.5); n += 1; }
    mass = float(std::max(1.0, n));
    const glm::dvec3 c = n > 0 ? s / n : glm::dvec3(sx, sy, sz) * 0.5;
    center = glm::ivec3(int(std::floor(c.x)), int(std::floor(c.y)), int(std::floor(c.z)));
    // инерция и рамка - относительно центра (середина вокселя center), в блоках
    const glm::vec3 cc = glm::vec3(center) + glm::vec3(0.5f);
    glm::dvec3 I(0);
    lo = glm::vec3(1e9f);
    hi = glm::vec3(-1e9f);
    probes.clear();
    std::vector<glm::vec3> surf;
    const auto filled = [&](int x, int y, int z) {
        return x >= 0 && y >= 0 && z >= 0 && x < sx && y < sy && z < sz && at(x, y, z) != world::AIR;
    };
    for (int y = 0; y < sy; ++y)
        for (int z = 0; z < sz; ++z)
            for (int x = 0; x < sx; ++x) {
                if (at(x, y, z) == world::AIR) continue;
                const glm::vec3 r = (glm::vec3(x + 0.5f, y + 0.5f, z + 0.5f) - cc) / FINE;
                I += glm::dvec3(r.y * r.y + r.z * r.z, r.x * r.x + r.z * r.z, r.x * r.x + r.y * r.y);
                lo = glm::min(lo, r - glm::vec3(0.5f / FINE));
                hi = glm::max(hi, r + glm::vec3(0.5f / FINE));
                const bool edge = !filled(x + 1, y, z) || !filled(x - 1, y, z) || !filled(x, y + 1, z) || !filled(x, y - 1, z) ||
                                  !filled(x, y, z + 1) || !filled(x, y, z - 1);
                if (edge) surf.push_back(r);
            }
    // тонкий предмет (палка в воксель) - инерция не меньше, чем у шарика в полвокселя: иначе вращение
    // вокруг длинной оси не гасится и тело дрожит
    const double floorI = mass * 0.02 / (FINE * FINE);
    inertia = glm::vec3(std::max(I.x, floorI), std::max(I.y, floorI), std::max(I.z, floorI));
    // точки касания: поверхность с шагом, не больше 96, плюс крайние
    const size_t stride = std::max<size_t>(1, surf.size() / 96);
    for (size_t i = 0; i < surf.size(); i += stride) probes.push_back(surf[i]);
    for (const glm::vec3& r : surf) {
        if (r.x - 0.5f / FINE <= lo.x + 1e-4f || r.x + 0.5f / FINE >= hi.x - 1e-4f || r.y - 0.5f / FINE <= lo.y + 1e-4f ||
            r.y + 0.5f / FINE >= hi.y - 1e-4f || r.z - 0.5f / FINE <= lo.z + 1e-4f || r.z + 0.5f / FINE >= hi.z - 1e-4f)
            if (probes.size() < 160) probes.push_back(r);
    }
}

std::string itemName(const Item& it) {
    static const char* SZ[3] = {"большой", "средний", "малый"};
    static const char* SZF[3] = {"большая", "средняя", "малая"};
    static const char* SZN[3] = {"широкое", "среднее", "малое"};
    switch (it.kind) {
        case ItemKind::AXE: return "топор";
        case ItemKind::SAW: return "пила";
        case ItemKind::DRAWKNIFE: return "скобель";
        case ItemKind::SHOVEL: return "лопата";
        case ItemKind::LOG: return std::string(SZN[int(it.size)]) + " бревно";
        case ItemKind::LIMB: return std::string(SZ[int(it.size)]) + " сук";
        case ItemKind::TWIG: return "ветка";
        case ItemKind::BARK: return "кора";
        case ItemKind::SNAG: return std::string(SZF[int(it.size)]) + " коряга";
        case ItemKind::STUMP: return "пень";
        case ItemKind::STONE: return "камень";
        case ItemKind::FIBER: return "волокно";
        case ItemKind::CLAY: return "ком глины";
        case ItemKind::HEAD_AXE: return "головка топора";
        case ItemKind::HEAD_KNIFE: return "лезвие ножа";
        case ItemKind::HEAD_SHOVEL: return "штык лопаты";
        case ItemKind::HEAD_SPEAR: return "наконечник копья";
        case ItemKind::KNIFE: return "нож";
        case ItemKind::SPEAR: return "копьё";
        case ItemKind::GRASS_CUT: return "охапка травы";
        case ItemKind::HAY: return "сено";
    }
    return "предмет";
}

Item makeTool(ItemKind kind, world::Material head) {
    using namespace world;
    std::shared_ptr<ItemModel> m;
    switch (kind) {
        case ItemKind::AXE: {
            // Каменный топор (образец пользователя): изогнутое топорище с утолщением к голове, кремнёвая
            // головка - тупой скруглённый обух и расширяющееся лезвие, - и кожаная обмотка полосами.
            // Воксель 1/64 блока: топорище 50 см - 32 вокселя.
            m = blank(26, 36, 9);
            m->res = 64;
            for (int y = 0; y < 36; ++y)
                for (int z = 0; z < 9; ++z)
                    for (int x = 0; x < 26; ++x) {
                        const float t = float(y) / 35.0f;
                        const float ax = 7.5f + 1.4f * std::sin(t * 3.1416f) - 0.8f * t, az = 4.5f;   // ось топорища - дугой
                        const float dx = x + 0.5f - ax, dz = z + 0.5f - az, d = std::sqrt(dx * dx + dz * dz);
                        float r = 1.55f + 0.75f * t;                     // к голове толще
                        if (y < 2) r = 2.1f;                              // утолщение на конце
                        const uint32_t hh0 = uint32_t(x * 73856093 ^ y * 19349663 ^ z * 83492791);
                        if (y < 31 && d <= r) m->at(x, y, z) = ((hh0 >> 7) % 5u) == 0 ? HANDLE2 : HANDLE;
                        // обмотка: поверх топорища под головкой, полосами наискось
                        if (y >= 21 && y <= 30 && d <= r + 0.9f)
                            m->at(x, y, z) = ((y + int(std::atan2(dz, dx) * 1.3f + 8)) & 3) < 2 ? LEATHER : LEATHER2;
                        // головка: вдоль x от обуха (u<0) к лезвию (u>0)
                        const float u = x + 0.5f - ax, v = y + 0.5f - 27.0f, w = z + 0.5f - az;
                        if (u >= -6.0f && u <= 17.0f) {
                            const float hh = u < 0 ? 4.2f * std::sqrt(std::max(0.0f, 1.0f - (u / 6.0f) * (u / 6.0f))) : 4.2f + 0.24f * u;
                            const float th = u < 0 ? 2.8f : 2.8f * (1.0f - u / 19.0f) + 0.45f;
                            const float rough = 0.35f * std::sin(x * 1.7f + y * 2.3f) * std::cos(z * 1.9f + x * 0.7f);
                            if (std::fabs(v) <= hh + rough && std::fabs(w) <= th) m->at(x, y, z) = head != world::AIR ? head : (((hh0 >> 9) % 3u) == 0 ? FLINT2 : FLINT);
                        }
                    }
            m->grip = glm::vec3(8.0f, 4.0f, 4.5f);
            break;
        }
        case ItemKind::SAW: {   // ножовка: полотно 60 см с зубьями и деревянная ручка с проёмом
            m = blank(12, 48, 3);
            m->res = 64;
            for (int y = 0; y < 10; ++y)
                for (int x = 0; x < 7; ++x)
                    for (int z = 0; z < 3; ++z)
                        if (!(x >= 2 && x <= 4 && y >= 3 && y <= 7)) m->at(x, y, z) = HANDLE;
            for (int y = 10; y < 48; ++y) {
                const int hgt = 9 - (y - 10) / 8;   // полотно сужается к концу
                for (int x = 1; x <= hgt; ++x) m->at(x, y, 1) = IRON;
                if (y % 2 == 0) m->at(hgt + 1, y, 1) = IRON;   // зубья
            }
            m->grip = glm::vec3(3.5f, 5.0f, 1.5f);
            break;
        }
        case ItemKind::DRAWKNIFE: {   // скобель: лезвие 45 см поперёк и две загнутые ручки
            m = blank(40, 14, 4);
            m->res = 64;
            for (int x = 6; x < 34; ++x) {
                for (int y = 9; y < 14; ++y) m->at(x, y, 1) = IRON;
                for (int y = 11; y < 14; ++y) m->at(x, y, 2) = IRON;   // обух лезвия толще
            }
            for (int s = 0; s < 2; ++s)
                for (int y = 0; y < 13; ++y)
                    for (int z = 0; z < 4; ++z)
                        for (int dx = 0; dx < 4; ++dx) {
                            const int x = s == 0 ? 2 + dx + y / 5 : 37 - dx - y / 5;
                            if (x >= 0 && x < 40) m->at(x, y, z) = HANDLE;
                        }
            m->grip = glm::vec3(4.0f, 4.0f, 2.0f);
            break;
        }
        case ItemKind::KNIFE: {   // Р99: каменный нож - рукоять 12 см, обмотка, лезвие 12 см
            m = blank(7, 18, 4);
            m->res = 64;
            const world::Material st = head != world::AIR ? head : FLINT;
            for (int y = 0; y < 18; ++y)
                for (int z = 0; z < 4; ++z)
                    for (int x = 0; x < 7; ++x) {
                        const float dx = x + 0.5f - 3.5f, dz = z + 0.5f - 2.0f;
                        if (y < 8 && dx * dx + dz * dz <= 2.6f) m->at(x, y, z) = HANDLE;
                        if (y >= 6 && y < 9 && dx * dx + dz * dz <= 4.2f) m->at(x, y, z) = (y & 1) ? LEATHER : LEATHER2;
                        // лезвие: плоское, сужается к острию
                        const float half = 2.6f * (1.0f - float(y - 8) / 10.0f);
                        if (y >= 8 && std::fabs(dx) <= half && std::fabs(dz) <= 0.9f) m->at(x, y, z) = st;
                    }
            m->grip = glm::vec3(3.5f, 3.0f, 2.0f);
            break;
        }
        case ItemKind::SPEAR: {   // Р99: копьё - древко 1.6 м, наконечник, обмотка
            m = blank(7, 112, 4);
            m->res = 64;
            const world::Material st = head != world::AIR ? head : FLINT;
            for (int y = 0; y < 112; ++y)
                for (int z = 0; z < 4; ++z)
                    for (int x = 0; x < 7; ++x) {
                        const float dx = x + 0.5f - 3.5f, dz = z + 0.5f - 2.0f;
                        if (y < 100 && dx * dx + dz * dz <= 1.7f) m->at(x, y, z) = HANDLE;
                        if (y >= 94 && y < 101 && dx * dx + dz * dz <= 3.2f) m->at(x, y, z) = (y & 1) ? LEATHER : LEATHER2;
                        const float half = 3.2f * std::min(1.0f, float(112 - y) / 8.0f);
                        if (y >= 99 && std::fabs(dx) <= half && std::fabs(dz) <= 1.1f) m->at(x, y, z) = st;
                    }
            m->grip = glm::vec3(3.5f, 40.0f, 2.0f);
            break;
        }
        default: {   // лопата: деревянный черенок 1.1 м, поперечная ручка, железный штык 25 см
            m = blank(18, 72, 4);
            m->res = 64;
            for (int y = 16; y < 68; ++y)
                for (int x = 7; x <= 10; ++x)
                    for (int z = 0; z < 4; ++z)
                        if ((x - 8.5f) * (x - 8.5f) + (z - 1.5f) * (z - 1.5f) <= 3.2f) m->at(x, y, z) = HANDLE;
            for (int y = 68; y < 72; ++y) for (int x = 3; x < 15; ++x) for (int z = 0; z < 4; ++z) m->at(x, y, z) = HANDLE;
            for (int y = 0; y < 18; ++y) {
                const int half = y < 3 ? 6 + y : 8;   // штык скруглён книзу
                for (int x = 9 - half; x < 9 + half; ++x) {
                    if (x < 0 || x >= 18) continue;
                    m->at(x, y, 1) = IRON;
                    if (y > 13) m->at(x, y, 2) = IRON;
                }
            }
            m->grip = glm::vec3(8.5f, 40.0f, 1.5f);
            break;
        }
    }
    if (kind == ItemKind::SHOVEL && head != world::AIR)   // Р99: каменный штык
        for (world::Material& v : m->v) if (v == IRON) v = head;
    m->finish();
    return Item{kind, ItemSize::SMALL, m, head};
}

void meshItem(const ItemModel& m, std::vector<ItemMeshChunk>& out) {
    out.clear();
    static thread_local world::ColumnHeights air;
    static thread_local bool ready = false;
    if (!ready) {
        for (auto& row : air.h) for (int32_t& v : row) v = INT32_MIN / 4;
        for (auto& row : air.water) for (int32_t& v : row) v = world::NO_WATER;
        ready = true;
    }
    auto c = std::make_unique<world::Chunk>();
    for (int cy = 0; cy * 32 < m.sy; ++cy)
        for (int cz = 0; cz * 32 < m.sz; ++cz)
            for (int cx = 0; cx * 32 < m.sx; ++cx) {
                c->pos = {cx, cy, cz};
                c->edited = true;
                c->voxels.fill(world::AIR);
                bool any = false;
                for (int y = 0; y < 32 && cy * 32 + y < m.sy; ++y)
                    for (int z = 0; z < 32 && cz * 32 + z < m.sz; ++z)
                        for (int x = 0; x < 32 && cx * 32 + x < m.sx; ++x) {
                            const world::Material v = m.at(cx * 32 + x, cy * 32 + y, cz * 32 + z);
                            if (v == world::AIR) continue;
                            c->at(x, y, z) = v;
                            any = true;
                        }
                if (!any) continue;
                ItemMeshChunk ch;
                world::MeshStats ms;
                world::buildMesh(*c, air, ch.verts, ms);
                if (ch.verts.empty()) continue;
                ch.off = glm::ivec3(cx * 32, cy * 32, cz * 32) - m.center;
                out.push_back(std::move(ch));
            }
}

uint32_t Items::spawn(const Item& it, glm::dvec3 pos, glm::quat rot, glm::vec3 vel, glm::vec3 ang, bool placed) {
    WorldItem w;
    w.id = next_++;
    w.item = it;
    w.body.pos = pos;
    w.body.rot = rot;
    w.body.vel = vel;
    w.body.ang = ang;
    w.placed = placed;
    w.body.asleep = placed;
    list.push_back(std::move(w));
    return list.back().id;
}

void Items::wake(glm::dvec3 at, double radius) {
    for (WorldItem& w : list)
        if (!w.placed && glm::distance(w.body.pos, at) < radius) { w.body.asleep = false; w.body.still = 0; }
}

void Items::step(world::VoxelQuery& q, float dt, glm::dvec3 near, double radius) {
    for (WorldItem& w : list) {
        if (w.placed || w.body.asleep) continue;
        if (glm::distance(w.body.pos, near) > radius) continue;
        const ItemModel& m = *w.item.model;
        stepRigid(w.body, m.mass, m.inertia, m.probes, [&q](int32_t x, int32_t y, int32_t z) { return q.solid(x, y, z); }, dt);
    }
}

int Items::pick(glm::dvec3 from, glm::dvec3 dir, double maxDist, double& tHit) const {
    int best = -1;
    tHit = maxDist;
    for (size_t i = 0; i < list.size(); ++i) {
        const WorldItem& w = list[i];
        const ItemModel& m = *w.item.model;
        const float FINE = float(m.res);
        // луч - в оси модели (мелкие воксели от угла модели), дальше шагами по модели
        const glm::mat3 Rt = glm::transpose(glm::mat3_cast(w.body.rot));
        const glm::vec3 o = Rt * glm::vec3(from - w.body.pos), d = Rt * glm::vec3(dir);
        // рамка: отсечь сразу
        double t0 = 0, t1 = tHit;
        bool miss = false;
        for (int a = 0; a < 3 && !miss; ++a) {
            const double lo = m.lo[a] - 0.05, hi = m.hi[a] + 0.05;
            if (std::fabs(d[a]) < 1e-9) { if (o[a] < lo || o[a] > hi) miss = true; continue; }
            double ta = (lo - o[a]) / d[a], tb = (hi - o[a]) / d[a];
            if (ta > tb) std::swap(ta, tb);
            t0 = std::max(t0, ta);
            t1 = std::min(t1, tb);
            if (t0 > t1) miss = true;
        }
        if (miss) continue;
        // по вокселям модели шагом в четверть мелкого вокселя; тонкий предмет берётся и с промахом в воксель
        const glm::vec3 cc = glm::vec3(m.center) + glm::vec3(0.5f);
        for (double t = t0; t <= t1; t += 0.25 / FINE) {
            const glm::vec3 p = (o + d * float(t)) * FINE + cc;
            bool hit = false;
            for (int dz = -1; dz <= 1 && !hit; ++dz)
                for (int dy = -1; dy <= 1 && !hit; ++dy)
                    for (int dx = -1; dx <= 1 && !hit; ++dx) {
                        const int x = int(std::floor(p.x)) + dx, y = int(std::floor(p.y)) + dy, z = int(std::floor(p.z)) + dz;
                        if (x < 0 || y < 0 || z < 0 || x >= m.sx || y >= m.sy || z >= m.sz) continue;
                        hit = m.at(x, y, z) != world::AIR;
                    }
            if (hit) {
                if (t < tHit) { tHit = t; best = int(i); }
                break;
            }
        }
    }
    return best;
}

}  // namespace game
