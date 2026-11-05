#include "game/tree_body.h"

#include <algorithm>
#include <cmath>

namespace game {

TreeBody makeTreeBody(uint64_t key, glm::ivec3 base, glm::ivec3 shift, const world::TreeSkel& skel, const TreeCut& cut, glm::vec2 fallDir) {
    TreeBody tb;
    tb.key = key;
    tb.base = base;
    tb.shift = shift;
    tb.skel = std::make_shared<world::TreeSkel>(skel);
    tb.cut = cut;
    tb.removed.assign(skel.branches.size(), 0);
    // корни остаются у пня
    for (size_t i = 1; i < tb.skel->nodes.size(); ++i)
        if (skel.branches[size_t(skel.nodes[i].branch)].root) tb.skel->nodes[i].r = 0;
    // Р96: высота на стволе, от которой растёт каждый сук (через предков) - по ней кусок ствола владеет сучьями
    tb.attachY.assign(skel.branches.size(), 0.0f);
    for (size_t b = 1; b < skel.branches.size(); ++b) {
        const world::TreeBranch& br = skel.branches[b];
        if (br.root || br.first < 0) continue;
        const world::TreeNode& at = skel.nodes[size_t(skel.nodes[size_t(br.first)].parent)];
        tb.attachY[b] = at.branch == 0 ? at.p[1] : tb.attachY[size_t(at.branch)];
    }
    tb.debarked.assign(256, 0);
    tb.yLo = cut.y0;
    tb.hinge = startFall(skel, cut, fallDir);
    tb.body.rot = tb.hinge.rot();
    reshape(tb);
    // поза из шарнира
    const glm::mat3 Rh = glm::mat3_cast(tb.hinge.rot());
    tb.body.pos = (tb.origin() + glm::dvec3(tb.hinge.hinge) + glm::dvec3(Rh * (tb.com - tb.hinge.hinge))) / 16.0;
    return tb;
}

void reshape(TreeBody& tb) {
    const world::TreeSkel& t = *tb.skel;
    const glm::vec3 oldCom = tb.com;
    const glm::dvec3 oldWorld = tb.body.pos;
    double m = 0;
    glm::dvec3 c(0);
    struct P { glm::vec3 q; float w; };
    std::vector<P> pts;
    for (size_t i = 1; i < t.nodes.size(); ++i) {
        const world::TreeNode& n = t.nodes[i];
        const world::TreeNode& pa = t.nodes[size_t(n.parent)];
        if (!tb.owns(n.branch, n.p[1]) || n.r <= 0) continue;
        const glm::vec3 a(pa.p[0], n.branch == 0 ? std::max(pa.p[1], tb.cut.y0) : pa.p[1], pa.p[2]), b(n.p[0], n.p[1], n.p[2]);
        const float r = (pa.r + n.r) * 0.5f, w = r * r * glm::distance(a, b) + 1e-3f;
        const glm::vec3 mid = (a + b) * 0.5f;
        m += w;
        c += glm::dvec3(mid) * double(w);
        pts.push_back({mid, w});
    }
    for (const world::TreeLeaf& l : t.leaves) {
        if (!tb.owns(l.branch, l.c[1])) continue;
        const float w = 0.05f * l.r * l.r;
        const glm::vec3 q(l.c[0], l.c[1], l.c[2]);
        m += w;
        c += glm::dvec3(q) * double(w);
        pts.push_back({q, w});
    }
    if (m <= 0) return;
    tb.com = glm::vec3(c / m);
    // масса - в «вокселях объёма»: r²·длина (мелкие воксели³), для удара важна лишь доля между телами
    tb.mass = float(m);
    glm::dvec3 I(0);
    for (const P& p : pts) {
        const glm::vec3 r = (p.q - tb.com) / 16.0f;   // блоки
        I += glm::dvec3(r.y * r.y + r.z * r.z, r.x * r.x + r.z * r.z, r.x * r.x + r.y * r.y) * double(p.w);
    }
    // дерево очень инертно: древесина плотная, а сучья и крона тянут момент наружу - втрое против скелета
    tb.inertia = glm::max(glm::vec3(I) * 3.0f, glm::vec3(tb.mass * 0.01f));
    // жёсткие точки: ствол (и торец реза), сучья толще трети ствола - пока толще половины своего основания,
    // и любые узлы толще 3 мелких вокселей (полено и на кривом суку держит)
    tb.probes.clear();
    tb.probes.push_back((glm::vec3(tb.cut.center.x, tb.cut.y0, tb.cut.center.y) - tb.com) / 16.0f);
    for (size_t i = 1; i < t.nodes.size(); ++i) {
        const world::TreeNode& n = t.nodes[i];
        if (!tb.owns(n.branch, n.p[1]) || n.r <= 0) continue;
        const world::TreeBranch& br = t.branches[size_t(n.branch)];
        bool rigid = n.branch == 0 || n.r >= 3.0f;
        if (!rigid && br.order == 1 && br.baseR > tb.cut.R * 0.3f) rigid = n.r > br.baseR * 0.5f;
        if (!rigid) continue;
        const glm::vec3 p(n.p[0], n.p[1], n.p[2]);
        tb.probes.push_back((p - tb.com) / 16.0f);
        // по бокам толстого узла - чтобы бревно катилось, а не стояло на линии
        if (n.r >= 2.0f) {
            const glm::vec3 side1 = glm::vec3(n.r, 0, 0), side2 = glm::vec3(0, 0, n.r), up = glm::vec3(0, n.r, 0);
            for (const glm::vec3& s : {side1, -side1, side2, -side2, up, -up}) tb.probes.push_back((p + s - tb.com) / 16.0f);
        }
    }
    // положение тела в мире не меняется: центр масс сместился внутри дерева
    if (oldCom != glm::vec3(0)) tb.body.pos = oldWorld + glm::dvec3(tb.R() * ((tb.com - oldCom) / 16.0f));
}

int hitBranch(const TreeBody& tb, glm::dvec3 from, glm::dvec3 dir, float maxBlocks, float& tBlocks, glm::vec3* localHit) {
    const world::TreeSkel& t = *tb.skel;
    const glm::vec3 o = tb.toLocal(from);
    const glm::vec3 d = glm::transpose(tb.R()) * glm::vec3(dir);
    int best = -1;
    float bestT = maxBlocks * 16.0f;
    for (size_t i = 1; i < t.nodes.size(); ++i) {
        const world::TreeNode& n = t.nodes[i];
        if (!tb.owns(n.branch, n.p[1]) || n.r <= 0) continue;
        const world::TreeNode& pa = t.nodes[size_t(n.parent)];
        const glm::vec3 a(pa.p[0], pa.p[1], pa.p[2]), b(n.p[0], n.p[1], n.p[2]);
        const float rr = std::max(std::max(pa.r, n.r), 0.9f);
        // ближайшая точка луча к отрезку - шагом по лучу внутри рамки отрезка
        const glm::vec3 lo = glm::min(a, b) - rr, hi = glm::max(a, b) + rr;
        float t0 = 0, t1 = bestT;
        bool miss = false;
        for (int k = 0; k < 3 && !miss; ++k) {
            if (std::fabs(d[k]) < 1e-6f) { if (o[k] < lo[k] || o[k] > hi[k]) miss = true; continue; }
            float ta = (lo[k] - o[k]) / d[k], tb2 = (hi[k] - o[k]) / d[k];
            if (ta > tb2) std::swap(ta, tb2);
            t0 = std::max(t0, ta);
            t1 = std::min(t1, tb2);
            if (t0 > t1) miss = true;
        }
        if (miss) continue;
        const glm::vec3 ab = b - a;
        for (float s = t0; s <= t1; s += 0.4f) {
            const glm::vec3 q = o + d * s;
            const float k = std::clamp(glm::dot(q - a, ab) / std::max(1e-6f, glm::dot(ab, ab)), 0.0f, 1.0f);
            const float r = std::max(pa.r + (n.r - pa.r) * k, 0.9f);
            if (glm::distance(q, a + ab * k) <= r) {
                if (s < bestT) { bestT = s; best = n.branch; if (localHit) *localHit = q; }
                break;
            }
        }
    }
    tBlocks = bestT / 16.0f;
    return best;
}

int limbHits(const TreeBody& tb, int branch) {
    const float d = 2.0f * tb.skel->branches[size_t(branch)].baseR;   // толщина у основания, мелкие воксели
    return std::max(1, int(d / 1.6f + 0.5f));
}

namespace {
// ветвь b и всё, что на ней растёт
std::vector<char> subtree(const world::TreeSkel& t, int b) {
    std::vector<char> in(t.branches.size(), 0);
    in[size_t(b)] = 1;
    for (size_t k = size_t(b) + 1; k < t.branches.size(); ++k) {
        const world::TreeBranch& br = t.branches[k];
        if (br.first < 0 || br.root) continue;
        const int owner = t.nodes[size_t(t.nodes[size_t(br.first)].parent)].branch;
        if (owner >= 0 && in[size_t(owner)]) in[k] = 1;
    }
    return in;
}
}  // namespace

bool severBranch(TreeBody& tb, int branch, Severed& out) {
    world::TreeSkel& t = *tb.skel;
    if (branch <= 0) return false;
    const std::vector<char> in = subtree(t, branch);
    // временный скелет: только отрубаемое; узел развилки - толщиной основания сука (срез)
    world::TreeSkel s = t;
    const world::TreeBranch& br = t.branches[size_t(branch)];
    const int attach = t.nodes[size_t(br.first)].parent;
    float lo[3] = {1e9f, 1e9f, 1e9f}, hi[3] = {-1e9f, -1e9f, -1e9f};
    bool twigs = false;
    for (size_t i = 0; i < s.nodes.size(); ++i) {
        const bool mine = i > 0 && in[size_t(t.nodes[i].branch)] && t.nodes[i].r > 0;
        if (int(i) == attach) { s.nodes[i].r = br.baseR; continue; }
        if (!mine) { s.nodes[i].r = 0; continue; }
        for (int a = 0; a < 3; ++a) { lo[a] = std::min(lo[a], t.nodes[i].p[a] - t.nodes[i].r - 2); hi[a] = std::max(hi[a], t.nodes[i].p[a] + t.nodes[i].r + 2); }
    }
    if (lo[0] > hi[0]) return false;
    for (int a = 0; a < 3; ++a) {
        lo[a] = std::min(lo[a], t.nodes[size_t(attach)].p[a] - br.baseR - 1);
        hi[a] = std::max(hi[a], t.nodes[size_t(attach)].p[a] + br.baseR + 1);
    }
    s.leaves.clear();
    for (const world::TreeLeaf& l : t.leaves)
        if (in[size_t(l.branch)] && tb.cut.leafStays(l.branch)) {   // Р103: опавшее не возвращается
            s.leaves.push_back(l);
            for (int a = 0; a < 3; ++a) { lo[a] = std::min(lo[a], l.c[a] - l.r * 1.3f - 1); hi[a] = std::max(hi[a], l.c[a] + l.r * 1.3f + 1); }
        }
    for (int a = 0; a < 3; ++a) { s.lo[a] = lo[a]; s.hi[a] = hi[a]; }
    for (size_t k = 0; k < t.branches.size(); ++k) if (in[k] && t.branches[k].order >= 2 && k != size_t(branch)) twigs = true;
    // воксели в плотную модель
    auto m = std::make_shared<ItemModel>();
    m->res = 16;
    const int x0 = int(std::floor(lo[0])), y0 = int(std::floor(lo[1])), z0 = int(std::floor(lo[2]));
    m->sx = int(std::ceil(hi[0])) - x0 + 1; m->sy = int(std::ceil(hi[1])) - y0 + 1; m->sz = int(std::ceil(hi[2])) - z0 + 1;
    if (size_t(m->sx) * m->sy * m->sz > 24u * 1024 * 1024) return false;   // слишком велико для одного предмета
    m->v.assign(size_t(m->sx) * m->sy * m->sz, world::AIR);
    const world::TreeIndex ix = world::treeIndex(s, 1);
    std::vector<world::Material> vox(32 * 32 * 32);
    std::vector<uint16_t> bo(32 * 32 * 32);
    const auto fd = [](int v) { return v >= 0 ? v / 32 : -((-v + 31) / 32); };
    for (int cz = fd(z0); cz <= fd(z0 + m->sz - 1); ++cz)
        for (int cy = fd(y0); cy <= fd(y0 + m->sy - 1); ++cy)
            for (int cx = fd(x0); cx <= fd(x0 + m->sx - 1); ++cx) {
                if (!world::treeChunk(s, ix, cx, cy, cz, true, true, vox.data(), bo.data())) continue;
                for (int y = 0; y < 32; ++y)
                    for (int z = 0; z < 32; ++z)
                        for (int x = 0; x < 32; ++x) {
                            const size_t k = (size_t(y) * 32 + z) * 32 + x;
                            if (vox[k] == world::AIR || bo[k] == 0xFFFF || !in[bo[k]]) continue;
                            const int lx = cx * 32 + x - x0, ly = cy * 32 + y - y0, lz = cz * 32 + z - z0;
                            if (lx < 0 || ly < 0 || lz < 0 || lx >= m->sx || ly >= m->sy || lz >= m->sz) continue;
                            m->at(lx, ly, lz) = vox[k];
                        }
            }
    m->finish();
    // класс по толщине у основания (Р90): сук 4 / 2 / 1 воксель, ветка - тонкий побег без своих ветвей
    const float d = 2.0f * br.baseR;
    Item it;
    it.model = m;
    if (d >= 3.5f) { it.kind = ItemKind::LIMB; it.size = ItemSize::LARGE; }
    else if (d >= 1.8f) { it.kind = ItemKind::LIMB; it.size = ItemSize::MEDIUM; }
    else if (!twigs && br.order >= 2) { it.kind = ItemKind::TWIG; it.size = ItemSize::SMALL; }
    else { it.kind = ItemKind::LIMB; it.size = ItemSize::SMALL; }
    out.item = it;
    out.rot = tb.body.rot;
    out.pos = tb.toWorld(glm::vec3(x0, y0, z0) + glm::vec3(m->center) + 0.5f);
    out.vel = tb.body.vel;
    // из тела - убрать
    for (size_t i = 1; i < t.nodes.size(); ++i) if (in[size_t(t.nodes[i].branch)]) t.nodes[i].r = 0;
    std::erase_if(t.leaves, [&](const world::TreeLeaf& l) { return in[size_t(l.branch)] != 0; });
    for (size_t k = 0; k < in.size(); ++k) if (in[k]) tb.removed[k] = 1;
    reshape(tb);
    tb.body.asleep = false;
    tb.body.still = 0;
    tb.meshDirty = true;
    return true;
}

std::shared_ptr<ItemModel> limbContour(const TreeBody& tb, int branch, glm::vec3& cornerLocal) {
    const world::TreeSkel& t = *tb.skel;
    const world::TreeBranch& br = t.branches[size_t(branch)];
    if (br.first < 0) return nullptr;
    const world::TreeNode& f = t.nodes[size_t(br.first)];
    const world::TreeNode& a = t.nodes[size_t(f.parent)];
    const glm::vec3 A(a.p[0], a.p[1], a.p[2]), F(f.p[0], f.p[1], f.p[2]);
    const glm::vec3 dir = glm::normalize(F - A);
    // кольцо там, где сук выходит из родителя: на расстоянии радиуса родителя от его оси
    const float s0 = a.r * 0.9f, R = std::max(br.baseR, 0.8f);
    const glm::vec3 c = A + dir * (s0 + 0.8f);
    const int half = int(std::ceil(R + 2.5f));
    auto m = std::make_shared<ItemModel>();
    m->res = 16;
    m->sx = m->sy = m->sz = 2 * half + 1;
    m->v.assign(size_t(m->sx) * m->sy * m->sz, world::AIR);
    const glm::vec3 corner = glm::floor(c) - glm::vec3(float(half));
    bool any = false;
    for (int y = 0; y < m->sy; ++y)
        for (int z = 0; z < m->sz; ++z)
            for (int x = 0; x < m->sx; ++x) {
                const glm::vec3 q = corner + glm::vec3(x + 0.5f, y + 0.5f, z + 0.5f) - A;
                const float along = glm::dot(q, dir) - s0;
                const float rad = glm::length(q - dir * glm::dot(q, dir));
                if (along < 0.0f || along > 1.6f) continue;
                if (rad < R - 0.2f || rad > R + 1.1f) continue;
                m->at(x, y, z) = world::HIGHLIGHT;
                any = true;
            }
    if (!any) return nullptr;
    m->finish();
    cornerLocal = corner;
    return m;
}

std::shared_ptr<ItemModel> trunkContour(const TreeBody& tb, float ym, glm::vec3& cornerLocal) {
    glm::vec2 c;
    float R;
    if (!trunkAt(*tb.skel, ym, c, R)) return nullptr;
    R = std::max(R, 0.8f);
    const int half = int(std::ceil(R + 2.5f));
    auto m = std::make_shared<ItemModel>();
    m->res = 16;
    m->sx = m->sz = 2 * half + 1;
    m->sy = 3;
    m->v.assign(size_t(m->sx) * m->sy * m->sz, world::AIR);
    const glm::vec3 corner(std::floor(c.x) - half, std::floor(ym) - 1, std::floor(c.y) - half);
    bool any = false;
    for (int y = 0; y < m->sy; ++y)
        for (int z = 0; z < m->sz; ++z)
            for (int x = 0; x < m->sx; ++x) {
                const glm::vec3 q = corner + glm::vec3(x + 0.5f, y + 0.5f, z + 0.5f);
                const float rad = glm::length(glm::vec2(q.x, q.z) - c);
                if (rad < R - 0.2f || rad > R + 1.1f) continue;
                m->at(x, y, z) = world::HIGHLIGHT;
                any = true;
            }
    if (!any) return nullptr;
    m->finish();
    cornerLocal = corner;
    return m;
}

bool splitBody(TreeBody& tb, float ym, TreeBody& upper) {
    if (ym <= tb.yLo + 8 || ym >= tb.yHi - 8) return false;
    upper = tb;
    upper.skel = std::make_shared<world::TreeSkel>(*tb.skel);
    upper.yLo = ym;
    tb.yHi = ym;
    for (TreeBody* b : {&tb, &upper}) {
        reshape(*b);
        b->body.asleep = false;
        b->body.still = 0;
        b->meshDirty = true;
        b->chopMark = -1;
        b->markHits = 0;
        b->chopBranch = -1;
    }
    return true;
}

bool isLog(const TreeBody& tb) {
    const world::TreeSkel& t = *tb.skel;
    float top = tb.yLo;
    for (size_t i = 1; i < t.nodes.size(); ++i) {
        const world::TreeNode& n = t.nodes[i];
        if (n.r <= 0 || !tb.owns(n.branch, n.p[1])) continue;
        if (n.branch != 0) return false;   // ещё есть сучья
        top = std::max(top, n.p[1]);
    }
    for (const world::TreeLeaf& l : t.leaves) if (tb.owns(l.branch, l.c[1])) return false;
    return std::min(top, tb.yHi) - tb.yLo <= 96.0f;
}

bool bodyToLog(const TreeBody& tb, Severed& out) {
    glm::vec3 corner;
    auto m = voxelModel(*tb.skel, [&tb](float x, float y, float z, int b) -> int {
        if (tb.cut.inNotch(x, y, z) || !tb.owns(b, y)) return 0;
        return tb.debarkedAt(y) ? 2 : 1;
    }, corner);
    if (!m) return false;
    glm::vec2 c;
    float R = 4;
    trunkAt(*tb.skel, (tb.yLo + std::min(tb.yHi, tb.yLo + 96.0f)) * 0.5f, c, R);
    const float d = 2.0f * R;
    Item it;
    it.kind = ItemKind::LOG;
    it.size = d >= 16.0f ? ItemSize::LARGE : (d >= 8.0f ? ItemSize::MEDIUM : ItemSize::SMALL);
    it.model = m;
    out.item = it;
    out.rot = tb.body.rot;
    out.pos = tb.toWorld(corner + glm::vec3(m->center) + 0.5f);
    out.vel = tb.body.vel;
    return true;
}

int debarkBody(TreeBody& tb, float y) {
    const int mi = int(std::floor((y - tb.cut.y0) / 16.0f));
    if (mi < 0 || mi >= int(tb.debarked.size()) || tb.debarked[size_t(mi)]) return 0;
    tb.debarked[size_t(mi)] = 1;
    tb.meshDirty = true;
    glm::vec2 c;
    float R = 2;
    trunkAt(*tb.skel, y, c, R);
    return std::max(1, int(2.0f * R / 4.0f + 0.5f));   // стружка на метр на каждую четверть метра толщины
}

int debarkItem(ItemModel& m, glm::vec3 p) {
    const int dims[3] = {m.sx, m.sy, m.sz};
    const int a = dims[0] >= dims[1] && dims[0] >= dims[2] ? 0 : (dims[1] >= dims[2] ? 1 : 2);
    const int s0 = int(std::floor(p[a] / 16.0f)) * 16;
    int changed = 0;
    for (int y = 0; y < m.sy; ++y)
        for (int z = 0; z < m.sz; ++z)
            for (int x = 0; x < m.sx; ++x) {
                const int c[3] = {x, y, z};
                if (c[a] < s0 || c[a] >= s0 + 16) continue;
                world::Material& v = m.at(x, y, z);
                if (v == world::BARK || v == world::BARK2) { v = world::WOOD; ++changed; }
            }
    if (!changed) return 0;
    const int thick = std::max(dims[(a + 1) % 3], dims[(a + 2) % 3]);
    return std::max(1, int(float(thick) / 4.0f + 0.5f));
}

Item makeBark() {
    auto m = std::make_shared<ItemModel>();
    m->res = 16;
    m->sx = 3; m->sy = 1; m->sz = 16;
    m->v.assign(48, world::BARK);
    m->at(0, 0, 0) = world::AIR;
    m->at(2, 0, 15) = world::AIR;
    m->finish();
    return Item{ItemKind::BARK, ItemSize::SMALL, m};
}

}  // namespace game
