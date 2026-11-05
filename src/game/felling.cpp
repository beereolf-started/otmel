#include "game/felling.h"

#include <algorithm>
#include <cmath>

#include <glm/geometric.hpp>
#include <glm/mat3x3.hpp>

namespace game {

bool TreeCut::inNotch(float x, float y, float z) const {
    if (!notched || depth <= 0) return false;
    const glm::vec2 h = glm::vec2(x, z) - center;
    // s - насколько точка выдвинута к игроку; зарубка - всё, что ближе к нему, чем R - depth
    const float s = glm::dot(h, dir);
    const float inner = R - depth;
    if (s <= inner) return false;
    // клин: раскрывается наружу под 45° вверх и вниз, у дна - щель в воксель
    return std::fabs(y - y0) <= (s - inner) * 0.85f + 0.6f;
}

float TreeCut::perHit() const {
    // молодой (R ~2-3) - около 4 ударов, взрослый (R ~4-5) - около 8, вековой (R ~8-10) - около 16
    const float n = std::clamp(R * 1.6f, 3.0f, 18.0f);
    return std::max(1.0f, 2.0f * R * 0.75f / n + 0.05f);
}

bool trunkAt(const world::TreeSkel& t, float y, glm::vec2& center, float& r) {
    for (size_t i = 1; i < t.nodes.size(); ++i) {
        const world::TreeNode& n = t.nodes[i];
        if (n.branch != 0) continue;
        const world::TreeNode& p = t.nodes[size_t(n.parent)];
        if (y < p.p[1] || y > n.p[1]) continue;
        const float k = (y - p.p[1]) / std::max(1e-3f, n.p[1] - p.p[1]);
        center = glm::vec2(p.p[0] + (n.p[0] - p.p[0]) * k, p.p[2] + (n.p[2] - p.p[2]) * k);
        r = p.r + (n.r - p.r) * k;
        return true;
    }
    return false;
}

bool hitTrunk(const world::TreeSkel& t, glm::vec3 o, glm::vec3 d, float maxT, float& tHit, glm::vec3& p) {
    // шаг в полвокселя; по отрезкам ствола и крупных сучьев (рубим ствол, но и попасть по нему надо честно)
    for (float s = 0; s <= maxT; s += 0.5f) {
        const glm::vec3 q = o + d * s;
        for (size_t i = 1; i < t.nodes.size(); ++i) {
            const world::TreeNode& n = t.nodes[i];
            if (n.branch != 0) continue;
            const world::TreeNode& pa = t.nodes[size_t(n.parent)];
            const glm::vec3 a(pa.p[0], pa.p[1], pa.p[2]), b(n.p[0], n.p[1], n.p[2]), ab = b - a;
            const float k = std::clamp(glm::dot(q - a, ab) / std::max(1e-6f, glm::dot(ab, ab)), 0.0f, 1.0f);
            const float rr = pa.r + (n.r - pa.r) * k;
            if (glm::distance(q, a + ab * k) <= rr) { tHit = s; p = q; return true; }
        }
    }
    return false;
}

Falling startFall(const world::TreeSkel& t, const TreeCut& cut, glm::vec2 fallDir) {
    Falling f;
    fallDir = glm::normalize(fallDir);
    {
        // крона заметно перевешивает в другую сторону (больше полуметра) - дерево падает туда, куда наклонено
        double w = 0;
        glm::dvec2 lean(0);
        for (size_t i = 1; i < t.nodes.size(); ++i) {
            const world::TreeNode& n = t.nodes[i];
            if (t.branches[size_t(n.branch)].root || n.p[1] < cut.y0) continue;
            const double m = double(n.r) * n.r;
            lean += glm::dvec2(n.p[0] - cut.center.x, n.p[2] - cut.center.y) * m;
            w += m;
        }
        if (w > 0) {
            const glm::vec2 l = glm::vec2(lean / w);
            if (glm::length(l) > 8.0f && glm::dot(l, fallDir) < 0) fallDir = glm::normalize(l);
        }
    }
    const glm::vec3 fd(fallDir.x, 0, fallDir.y);
    // шарнир - недоруб на дальней от зарубки стороне
    f.hinge = glm::vec3(cut.center.x, cut.y0, cut.center.y) + fd * (cut.R * 0.7f);
    f.axis = glm::normalize(glm::cross(glm::vec3(0, 1, 0), fd));
    // масса и инерция: отрезки выше реза, масса ~ r²·длина
    double m = 0;
    glm::dvec3 c(0);
    std::vector<std::pair<glm::vec3, float>> pts;
    for (size_t i = 1; i < t.nodes.size(); ++i) {
        const world::TreeNode& n = t.nodes[i];
        const world::TreeNode& pa = t.nodes[size_t(n.parent)];
        if (t.branches[size_t(n.branch)].root) continue;
        if (n.p[1] < cut.y0) continue;
        const glm::vec3 a(pa.p[0], std::max(pa.p[1], cut.y0), pa.p[2]), b(n.p[0], n.p[1], n.p[2]);
        const float len = glm::distance(a, b), r = (pa.r + n.r) * 0.5f;
        const float w = r * r * len;
        const glm::vec3 mid = (a + b) * 0.5f - f.hinge;
        m += w;
        c += glm::dvec3(mid) * double(w);
        pts.push_back({mid, w});
    }
    // листва - тоже вес (немного)
    for (const world::TreeLeaf& l : t.leaves) {
        if (l.c[1] < cut.y0) continue;
        const glm::vec3 mid = glm::vec3(l.c[0], l.c[1], l.c[2]) - f.hinge;
        const float w = 0.05f * l.r * l.r;
        m += w;
        c += glm::dvec3(mid) * double(w);
        pts.push_back({mid, w});
    }
    f.com = m > 0 ? glm::vec3(c / m) : glm::vec3(0, 100, 0);
    double I = 0;
    for (const auto& [q, w] : pts) {
        const float along = glm::dot(q, f.axis);
        I += double(w) * double(glm::dot(q, q) - along * along);
    }
    f.I = float(std::max(1.0, I / std::max(1e-6, m)));
    // точки касания - только ЖЁСТКИЕ: ствол и основания крупных сучьев. Тонкие ветви и крона землю не держат:
    // они сминаются (crushFallen), и дерево ложится, пока не упрётся стволом или сучьями - хоть наклонно.
    for (size_t i = 1; i < t.nodes.size(); ++i) {
        const world::TreeNode& n = t.nodes[i];
        const world::TreeBranch& br = t.branches[size_t(n.branch)];
        if (br.root || n.p[1] < cut.y0 + 4) continue;
        bool rigid = n.branch == 0;
        if (!rigid && br.order == 1 && br.baseR > cut.R * 0.3f) {
            // первые узлы сука - пока он толще половины своего основания
            rigid = n.r > br.baseR * 0.5f;
        }
        if (rigid) f.probes.push_back(glm::vec3(n.p[0], n.p[1], n.p[2]) - f.hinge);
    }
    f.theta = 0.015f;   // толчок: недоруб сломался
    f.omega = 0.25f;
    return f;
}

void stepFall(Falling& f, world::VoxelQuery& q, glm::dvec3 originFine, float dt) {
    if (f.done) return;
    constexpr float G = 28.0f * 16.0f;   // тяжесть, мелкие воксели/с²
    const int sub = 8;
    const float h = std::min(dt, 0.05f) / sub;
    for (int s = 0; s < sub && !f.done; ++s) {
        const glm::mat3 R = glm::mat3_cast(f.rot());
        // момент тяжести на единицу массы: (r × g)·ось
        const glm::vec3 r = R * f.com;
        const float tau = glm::dot(glm::cross(r, glm::vec3(0, -G, 0)), f.axis);
        f.omega += tau / f.I * h;
        f.omega *= 0.9995f;
        const float prev = f.theta;
        f.theta += f.omega * h;
        // на зарубку назад не валится: недоруб держит, дерево сползает только вперёд
        if (f.theta < 0.01f) { f.theta = 0.01f; f.omega = std::max(f.omega, 0.05f); }
        // касание земли: узел внутри твёрдого блока
        const glm::mat3 R2 = glm::mat3_cast(f.rot());
        const glm::dvec3 hw = originFine + glm::dvec3(f.hinge);
        bool contact = false;
        for (const glm::vec3& p : f.probes) {
            const glm::dvec3 w = (hw + glm::dvec3(R2 * p)) / 16.0;
            if (q.solid(int32_t(std::floor(w.x)), int32_t(std::floor(w.y)), int32_t(std::floor(w.z)))) { contact = true; break; }
        }
        if (contact) {
            f.theta = prev;
            f.omega = -f.omega * 0.18f;   // отскок кроны
            if (std::fabs(f.omega) < 0.25f) { f.omega = 0; f.done = true; }
        }
        if (f.theta > 1.75f) { f.theta = 1.75f; f.omega = 0; f.done = true; }   // ниже горизонта не падает
    }
}

std::shared_ptr<world::TreeSkel> crushFallen(const world::TreeSkel& t, const Falling& f, world::VoxelQuery& q, glm::dvec3 originFine,
                                             float y0) {
    auto out = std::make_shared<world::TreeSkel>(t);
    const glm::mat3 R = glm::mat3_cast(f.rot()), Rt = glm::transpose(R);
    const glm::dvec3 hw = originFine + glm::dvec3(f.hinge);
    // сдвиг узла в мире (мелкие воксели): от родителя наследуется, сверх того - подъём на поверхность
    std::vector<glm::vec3> delta(t.nodes.size(), glm::vec3(0));
    const auto solid = [&](glm::dvec3 w) {
        return q.solid(int32_t(std::floor(w.x / 16.0)), int32_t(std::floor(w.y / 16.0)), int32_t(std::floor(w.z / 16.0)));
    };
    for (size_t i = 1; i < t.nodes.size(); ++i) {
        const world::TreeNode& n = t.nodes[i];
        if (t.branches[size_t(n.branch)].root || n.p[1] < y0) continue;
        delta[i] = delta[size_t(n.parent)];
        glm::dvec3 w = hw + glm::dvec3(R * (glm::vec3(n.p[0], n.p[1], n.p[2]) - f.hinge)) + glm::dvec3(delta[i]);
        if (!solid(w)) continue;
        // вверх до первого свободного блока, и ещё на свою толщину
        double y = std::floor(w.y / 16.0);
        for (int k = 0; k < 64 && q.solid(int32_t(std::floor(w.x / 16.0)), int32_t(y), int32_t(std::floor(w.z / 16.0))); ++k) y += 1;
        const float lift = float(y * 16.0 + n.r - w.y);
        delta[i].y += std::max(0.0f, lift);
    }
    for (size_t i = 1; i < t.nodes.size(); ++i) {
        if (delta[i] == glm::vec3(0)) continue;
        const glm::vec3 d = Rt * delta[i];   // сдвиг - в оси дерева
        for (int a = 0; a < 3; ++a) out->nodes[i].p[a] += d[a];
    }
    // листва - вместе с последним узлом своей ветви
    for (world::TreeLeaf& l : out->leaves) {
        const int last = t.branches[size_t(l.branch)].last;
        if (last < 0 || delta[size_t(last)] == glm::vec3(0)) continue;
        const glm::vec3 d = Rt * delta[size_t(last)];
        for (int a = 0; a < 3; ++a) l.c[a] += d[a];
    }
    // рамка могла вырасти
    for (const world::TreeNode& n : out->nodes)
        for (int a = 0; a < 3; ++a) { out->lo[a] = std::min(out->lo[a], n.p[a] - n.r * 1.3f - 2); out->hi[a] = std::max(out->hi[a], n.p[a] + n.r * 1.3f + 2); }
    for (const world::TreeLeaf& l : out->leaves)
        for (int a = 0; a < 3; ++a) { out->lo[a] = std::min(out->lo[a], l.c[a] - l.r * 1.3f - 1); out->hi[a] = std::max(out->hi[a], l.c[a] + l.r * 1.3f + 1); }
    return out;
}

}  // namespace game
