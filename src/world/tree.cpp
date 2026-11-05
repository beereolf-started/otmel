#include "world/tree.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace world {
namespace {

struct V3 {
    float x = 0, y = 0, z = 0;
    V3 operator+(V3 o) const { return {x + o.x, y + o.y, z + o.z}; }
    V3 operator-(V3 o) const { return {x - o.x, y - o.y, z - o.z}; }
    V3 operator*(float k) const { return {x * k, y * k, z * k}; }
};
float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
float len(V3 a) { return std::sqrt(dot(a, a)); }
V3 norm(V3 a) { const float l = len(a); return l > 1e-6f ? a * (1.0f / l) : V3{0, 1, 0}; }
const V3 UP{0, 1, 0};
V3 P(const TreeNode& n) { return {n.p[0], n.p[1], n.p[2]}; }

struct Rng {
    uint64_t s;
    explicit Rng(uint64_t seed) : s(0x9E3779B97F4A7C15ull ^ (seed * 0xD1B54A32D192ED03ull)) { next(); }
    uint32_t next() {
        s += 0x9E3779B97F4A7C15ull;
        uint64_t z = s;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return uint32_t((z ^ (z >> 31)) >> 32);
    }
    float f() { return float(next() >> 8) / 16777216.0f; }
    float r(float a, float b) { return a + (b - a) * f(); }
    V3 unit() {
        const float z = r(-1, 1), a = r(0, 6.2831853f), q = std::sqrt(1 - z * z);
        return {q * std::cos(a), z, q * std::sin(a)};
    }
};

float hash3(int x, int y, int z, uint32_t seed) {
    uint32_t h = uint32_t(x) * 0x8DA6B343u ^ uint32_t(y) * 0xD8163841u ^ uint32_t(z) * 0xCB1AB31Fu ^ seed;
    h ^= h >> 13; h *= 0x5BD1E995u; h ^= h >> 15;
    return float(h & 0xFFFFFF) / 8388607.5f - 1.0f;
}
float noise3(V3 p, uint32_t seed) {
    const int ix = int(std::floor(p.x)), iy = int(std::floor(p.y)), iz = int(std::floor(p.z));
    float fx = p.x - ix, fy = p.y - iy, fz = p.z - iz;
    fx = fx * fx * (3 - 2 * fx); fy = fy * fy * (3 - 2 * fy); fz = fz * fz * (3 - 2 * fz);
    const auto l = [](float a, float b, float t) { return a + (b - a) * t; };
    const auto h = [&](int dx, int dy, int dz) { return hash3(ix + dx, iy + dy, iz + dz, seed); };
    return l(l(l(h(0, 0, 0), h(1, 0, 0), fx), l(h(0, 1, 0), h(1, 1, 0), fx), fy),
             l(l(h(0, 0, 1), h(1, 0, 1), fx), l(h(0, 1, 1), h(1, 1, 1), fx), fy), fz);
}

// Параметры возраста (мелкие воксели, 16 на блок)
struct AgeDef {
    float hMin, hMax;        // высота дерева
    float forkMin, forkMax;  // высота развилки (низ кроны), доля высоты
    float crownW;            // радиус кроны по горизонтали, доля высоты
    float rBase;             // радиус ствола у комля
    float step;              // шаг роста
    int attractors;          // точек в облаке кроны
    float leafMin, leafMax;  // радиус комка листвы
    float rootLen;           // длина корня
    float tip;               // радиус кончика побега
};
const AgeDef AGES[3] = {
    {96, 128, 0.22f, 0.34f, 0.66f, 2.0f, 6, 320, 7, 11, 40, 0.5f},       // молодой: 6-8 м, ствол ~25 см
    {192, 256, 0.24f, 0.34f, 0.55f, 4.0f, 9, 700, 9, 14, 80, 0.55f},     // взрослый: 12-16 м, ствол ~50 см
    // вековой: 20-26 м, ствол ~1 м (образ - дуб-точка синхронизации из AC3), крона раскидистая, низкие мощные сучья
    {320, 416, 0.14f, 0.22f, 0.62f, 8.0f, 12, 1200, 11, 17, 150, 0.6f},
};

struct Grow {
    Rng rng;
    std::vector<TreeNode> nodes;
    std::vector<V3> dir;       // направление роста узла
    std::vector<float> zig;    // знак зигзага
    std::vector<int> kids;     // число детей
    explicit Grow(uint64_t seed) : rng(seed) {}
    int add(V3 p, int parent, V3 d, float z) {
        nodes.push_back({{p.x, p.y, p.z}, 0, parent, -1});
        dir.push_back(d);
        zig.push_back(z);
        kids.push_back(0);
        if (parent >= 0) ++kids[size_t(parent)];
        return int(nodes.size()) - 1;
    }
};

}  // namespace

int treeLeafScale(TreeAge age) { return age == TreeAge::YOUNG ? 1 : 2; }

std::shared_ptr<const TreeSkel> growTree(uint32_t seed, TreeAge age) {
    auto t = std::make_shared<TreeSkel>();
    t->age = age;
    t->seed = seed;
    const AgeDef& A = AGES[int(age)];
    Grow g(uint64_t(seed) * 3 + uint64_t(age));
    Rng& rng = g.rng;
    const float H = rng.r(A.hMin, A.hMax);
    const float fork = H * rng.r(A.forkMin, A.forkMax);
    const float D = A.step;

    // ─── ствол до развилки: наклон и изгиб (стволы в поле не стоят столбом)
    const float leanAz = rng.r(0, 6.2831853f), lean = rng.r(0.05f, 0.22f);
    const V3 leanDir{std::cos(leanAz), 0, std::sin(leanAz)};
    V3 d = norm(UP + leanDir * lean);
    V3 p{0, 0, 0};
    int at = g.add(p, -1, d, 1);
    const V3 bendDir = norm(cross(UP, leanDir) * rng.r(-1, 1) + leanDir * rng.r(-0.6f, 0.4f));
    while (p.y < fork) {
        const float t = p.y / std::max(1.0f, fork);
        // изгиб: в середине ствол уходит вбок, к развилке выпрямляется к свету
        d = norm(d + bendDir * (0.10f * std::sin(t * 3.1416f)) + UP * 0.08f + rng.unit() * 0.03f);
        p = p + d * D;
        at = g.add(p, at, d, 1);
    }

    // ─── облако кроны: перекошенный эллипсоид с вмятинами, центр сдвинут вбок
    const float Rc = H * A.crownW * rng.r(0.85f, 1.15f);
    const float Ry = (H - fork) * 0.5f;   // крона дуба шире своей высоты, верх приплюснут
    const float sideAz = leanAz + rng.r(-1.2f, 1.2f);
    const V3 C = V3{p.x, fork + Ry * 0.95f, p.z} + V3{std::cos(sideAz), 0, std::sin(sideAz)} * (Rc * rng.r(0.1f, 0.3f));
    const uint32_t ns = seed * 0x27D4EB2Du + 0x165667B1u;
    std::vector<V3> pts;
    pts.reserve(size_t(A.attractors));
    for (int tries = 0; int(pts.size()) < A.attractors && tries < A.attractors * 40; ++tries) {
        const V3 u{rng.r(-1, 1), rng.r(-1, 1), rng.r(-1, 1)};
        const float l = len(u);
        if (l > 1.0f) continue;
        // вмятины и выступы кроны: радиус гуляет низкочастотным шумом по направлению
        const V3 nu = norm(u);
        // перекос: со стороны sideAz крона полнее, с противоположной - урезана (там «соседи», нет света)
        const float lit = 0.5f + 0.5f * (nu.x * std::cos(sideAz) + nu.z * std::sin(sideAz));
        const float bump = (0.72f + 0.42f * lit) * (1.0f + 0.5f * noise3(nu * 1.7f + V3{3.1f, 7.7f, 1.3f}, ns));
        if (l > bump) continue;
        V3 q{C.x + u.x * Rc, C.y + u.y * Ry, C.z + u.z * Rc};
        // низ кроны - плоский, у дуба нижние сучья раскинуты вширь и свисают
        if (q.y < fork * 0.85f) continue;
        pts.push_back(q);
    }

    // ─── колонизация: узлы растут к ближайшим ещё не занятым точкам
    const float di = D * 7.0f, dk = D * 1.6f;
    std::vector<char> alive(pts.size(), 1);
    std::vector<V3> pull;
    std::vector<int> pullN;
    const auto cellOf = [&](V3 q) {
        return (uint64_t(uint32_t(int(std::floor(q.x / di)) + 4096)) << 42) ^ (uint64_t(uint32_t(int(std::floor(q.y / di)) + 4096)) << 21) ^
               uint64_t(uint32_t(int(std::floor(q.z / di)) + 4096));
    };
    std::unordered_map<uint64_t, std::vector<int>> grid;
    for (int i = 0; i < int(g.nodes.size()); ++i) grid[cellOf(P(g.nodes[size_t(i)]))].push_back(i);
    int left = int(pts.size());
    for (int iter = 0; iter < 220 && left > 0; ++iter) {
        pull.assign(g.nodes.size(), V3{});
        pullN.assign(g.nodes.size(), 0);
        for (size_t k = 0; k < pts.size(); ++k) {
            if (!alive[k]) continue;
            const V3 q = pts[k];
            int best = -1;
            float bd = di * di;
            const int ix = int(std::floor(q.x / di)), iy = int(std::floor(q.y / di)), iz = int(std::floor(q.z / di));
            for (int dz = -1; dz <= 1; ++dz)
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        const uint64_t key = (uint64_t(uint32_t(ix + dx + 4096)) << 42) ^ (uint64_t(uint32_t(iy + dy + 4096)) << 21) ^
                                             uint64_t(uint32_t(iz + dz + 4096));
                        const auto it = grid.find(key);
                        if (it == grid.end()) continue;
                        for (int n : it->second) {
                            const V3 e = q - P(g.nodes[size_t(n)]);
                            const float d2 = dot(e, e);
                            if (d2 < bd) { bd = d2; best = n; }
                        }
                    }
            if (best < 0) continue;
            if (bd < dk * dk) { alive[k] = 0; --left; continue; }
            pull[size_t(best)] = pull[size_t(best)] + norm(q - P(g.nodes[size_t(best)]));
            ++pullN[size_t(best)];
        }
        bool grew = false;
        const size_t n0 = g.nodes.size();
        for (size_t n = 0; n < n0; ++n) {
            if (!pullN[n]) continue;
            // узел, давший уже трёх детей, больше не ветвится
            if (g.kids[n] >= 3) continue;
            const V3 pd = g.dir[n];
            V3 nd = norm(pull[n]);
            // ДУБОВЫЙ ЗИГЗАГ: поперёк роста, знак меняется на каждом шаге
            const V3 side = norm(cross(nd, std::fabs(nd.y) < 0.95f ? UP : V3{1, 0, 0}));
            const float z = -g.zig[n];
            const float horiz = 1.0f - std::fabs(nd.y);
            nd = norm(nd + side * (z * 0.42f) + rng.unit() * 0.12f + pd * 0.35f
                      // провисание: пологий побег клонится вниз, сильнее - чем дальше от ствола
                      + UP * (-0.36f * horiz * std::min(1.0f, len(P(g.nodes[n]) - V3{p.x, P(g.nodes[n]).y, p.z}) / Rc)));
            const V3 q = P(g.nodes[n]) + nd * D;
            const int id = g.add(q, int(n), nd, z);
            grid[cellOf(q)].push_back(id);
            grew = true;
        }
        if (!grew) break;
    }

    // ─── толщина: трубочная модель, от кончиков к комлю, затем подгон под толщину комля
    const size_t N = g.nodes.size();
    std::vector<float> raw(N, 0.0f);
    constexpr float E = 2.1f;   // ниже показатель - толще сучья относительно ствола: у дуба сучья мощные
    for (size_t i = N; i-- > 0;) {
        if (g.kids[i] == 0) raw[i] = std::max(raw[i], 1.0f);
        const float r = std::pow(raw[i], 1.0f / E);
        g.nodes[i].r = r;
        if (g.nodes[i].parent >= 0) raw[size_t(g.nodes[i].parent)] += std::pow(r, E);
    }
    const float k = A.rBase * rng.r(0.85f, 1.15f) / std::max(1e-3f, g.nodes[0].r);
    for (size_t i = 0; i < N; ++i) {
        float r = std::max(A.tip, g.nodes[i].r * k);
        // корневой наплыв у земли
        if (g.nodes[i].p[1] < A.rBase * 3) r *= 1.0f + 0.35f * std::exp(-std::max(0.0f, g.nodes[i].p[1]) / (A.rBase * 1.2f));
        g.nodes[i].r = r;
    }

    // ─── ветви: продолжение - самый толстый ребёнок
    std::vector<std::vector<int>> ch(N);
    for (size_t i = 1; i < N; ++i) ch[size_t(g.nodes[i].parent)].push_back(int(i));
    std::vector<int> main(N, -1);
    for (size_t i = 0; i < N; ++i) {
        float br = -1;
        for (int c : ch[i]) if (g.nodes[size_t(c)].r > br) { br = g.nodes[size_t(c)].r; main[i] = c; }
    }
    t->branches.push_back({-1, 0, 0, 0, false, false, g.nodes[0].r});
    g.nodes[0].branch = 0;
    for (size_t i = 0; i < N; ++i) {   // родитель всегда раньше ребёнка
        for (int c : ch[i]) {
            TreeNode& cn = g.nodes[size_t(c)];
            if (c == main[i]) {
                cn.branch = g.nodes[i].branch;
                t->branches[size_t(cn.branch)].last = c;
            } else {
                const TreeBranch& pb = t->branches[size_t(g.nodes[i].branch)];
                t->branches.push_back({g.nodes[i].branch, c, c, pb.order + 1, false, false, cn.r});
                cn.branch = int(t->branches.size()) - 1;
            }
        }
    }

    // ─── очищение снизу: сучья ниже середины кроны часто отмерли - обломки
    std::vector<char> drop(N, 0);
    for (size_t b = 1; b < t->branches.size(); ++b) {
        TreeBranch& br = t->branches[b];
        const TreeNode& base = g.nodes[size_t(br.first)];
        const float lowness = 1.0f - std::clamp((base.p[1] - fork) / std::max(1.0f, Ry), 0.0f, 1.0f);
        const bool parentTrunk = br.order == 1;
        if (rng.f() < (parentTrunk ? 0.55f : 0.08f) * lowness) {
            br.dead = true;
            // обломок: от ветви остаётся 1-3 узла, всё дальше отпадает
            const int keep = 1 + int(rng.f() * 3);
            int cnt = 0;
            for (int n = br.first; n >= 0 && n < int(N);) {
                ++cnt;
                int next = -1;
                for (int c : ch[size_t(n)]) if (c == main[size_t(n)]) next = c;
                if (cnt > keep) drop[size_t(n)] = 1;
                if (next < 0) break;
                n = next;
            }
        }
    }
    // отпавшее - вместе со всем, что на нём росло
    for (size_t i = 1; i < N; ++i) if (drop[size_t(g.nodes[i].parent)]) drop[i] = 1;
    // уплотнить узлы, переписав родителей и ветви
    std::vector<int> remap(N, -1);
    for (size_t i = 0; i < N; ++i) {
        if (drop[i]) continue;
        remap[i] = int(t->nodes.size());
        TreeNode n = g.nodes[i];
        n.parent = n.parent >= 0 ? remap[size_t(n.parent)] : -1;
        t->nodes.push_back(n);
    }
    for (TreeBranch& br : t->branches) {
        br.first = remap[size_t(br.first)];
        int last = -1;
        for (int n = br.last; n >= 0; n = g.nodes[size_t(n)].parent)
            if (remap[size_t(n)] >= 0 && g.nodes[size_t(n)].branch == int(&br - t->branches.data())) { last = remap[size_t(n)]; break; }
        br.last = last;
        if (br.first >= 0) br.baseR = t->nodes[size_t(br.first)].r;
    }

    // ─── корни: от комля наружу и вниз, ветвятся раз-другой
    const int nr = 4 + int(rng.f() * 4);
    t->roots = nr;
    for (int kk = 0; kk < nr; ++kk) {
        const float az = 6.2831853f * kk / nr + rng.r(-0.35f, 0.35f), b = rng.r(0.25f, 0.6f);
        V3 rd = norm(V3{std::cos(az), 0, std::sin(az)} * std::cos(b) + UP * -std::sin(b));
        const int bi = int(t->branches.size());
        t->branches.push_back({-1, -1, -1, 1, true, false, 0});
        float r = A.rBase * rng.r(0.45f, 0.7f);
        V3 q{0, A.rBase * 0.5f, 0};
        int parent = 0;
        const float L = A.rootLen * rng.r(0.6f, 1.1f);
        const int steps = std::max(2, int(L / D));
        for (int s = 0; s < steps; ++s) {
            rd = norm(rd + rng.unit() * 0.25f + UP * -0.05f);
            if (rd.y > -0.05f) rd = norm(rd + UP * -0.2f);
            q = q + rd * D;
            r = std::max(A.tip, r * 0.86f);
            t->nodes.push_back({{q.x, q.y, q.z}, r, parent, bi});
            parent = int(t->nodes.size()) - 1;
            if (t->branches[size_t(bi)].first < 0) { t->branches[size_t(bi)].first = parent; t->branches[size_t(bi)].baseR = r; }
            t->branches[size_t(bi)].last = parent;
        }
    }

    // ─── листва: комки у кончиков и у последних узлов живых ветвей (не у ствола, не у обломков)
    for (size_t i = 1; i < t->nodes.size(); ++i) {
        const TreeNode& n = t->nodes[i];
        const TreeBranch& br = t->branches[size_t(n.branch)];
        if (br.root || br.dead || br.order == 0) continue;
        if (n.r > A.tip * 3.0f) continue;   // листва - на тонких побегах
        if (rng.f() > 0.38f) continue;   // Р102: листвы меньше - не мешает
        const V3 c = P(n) + UP * rng.r(0.5f, 2.0f) + rng.unit() * 1.5f;
        t->leaves.push_back({{c.x, c.y, c.z}, rng.r(A.leafMin, A.leafMax), n.branch});
    }

    // ─── рамка
    float lo[3] = {1e9f, 1e9f, 1e9f}, hi[3] = {-1e9f, -1e9f, -1e9f};
    const auto grow = [&](const float* q, float r) {
        for (int a = 0; a < 3; ++a) { lo[a] = std::min(lo[a], q[a] - r); hi[a] = std::max(hi[a], q[a] + r); }
    };
    for (const TreeNode& n : t->nodes) grow(n.p, n.r * 1.3f + 2);
    for (const TreeLeaf& l : t->leaves) grow(l.c, l.r * 1.3f + 1);
    for (int a = 0; a < 3; ++a) { t->lo[a] = lo[a]; t->hi[a] = hi[a]; }
    t->height = hi[1];
    t->crownR = std::max(std::max(-lo[0], hi[0]), std::max(-lo[2], hi[2]));
    return t;
}

void treeChunkRange(const TreeSkel& t, int scale, int lo[3], int hi[3]) {
    const float S = 32.0f * float(scale);
    for (int a = 0; a < 3; ++a) {
        lo[a] = int(std::floor(t.lo[a] / S));
        hi[a] = int(std::floor(t.hi[a] / S));
    }
}

TreeIndex treeIndex(const TreeSkel& t, int scale) {
    TreeIndex ix;
    ix.scale = scale;
    const float iS = 1.0f / float(scale * 32);
    const auto put = [&](const float* lo, const float* hi, bool seg, int id) {
        const int x0 = int(std::floor(lo[0] * iS)), x1 = int(std::floor(hi[0] * iS));
        const int y0 = int(std::floor(lo[1] * iS)), y1 = int(std::floor(hi[1] * iS));
        const int z0 = int(std::floor(lo[2] * iS)), z1 = int(std::floor(hi[2] * iS));
        for (int z = z0; z <= z1; ++z)
            for (int y = y0; y <= y1; ++y)
                for (int x = x0; x <= x1; ++x) {
                    auto& c = ix.cells[TreeIndex::key(x, y, z)];
                    (seg ? c.segs : c.leaves).push_back(id);
                }
    };
    for (size_t i = 1; i < t.nodes.size(); ++i) {
        const TreeNode& n = t.nodes[i];
        const TreeNode& p = t.nodes[size_t(n.parent)];
        const float rm = std::max(n.r, p.r) * 1.1f + float(scale) + 1.0f;
        float lo[3], hi[3];
        for (int a = 0; a < 3; ++a) { lo[a] = std::min(n.p[a], p.p[a]) - rm; hi[a] = std::max(n.p[a], p.p[a]) + rm; }
        put(lo, hi, true, int(i));
    }
    for (size_t i = 0; i < t.leaves.size(); ++i) {
        const TreeLeaf& l = t.leaves[i];
        const float rm = l.r * 1.2f + float(scale) + 1.0f;
        float lo[3], hi[3];
        for (int a = 0; a < 3; ++a) { lo[a] = l.c[a] - rm; hi[a] = l.c[a] + rm; }
        put(lo, hi, false, int(i));
    }
    return ix;
}

bool treeChunk(const TreeSkel& t, const TreeIndex& ix, int cx, int cy, int cz, bool wood, bool leaves, Material* out,
               uint16_t* branchOut) {
    const int scale = ix.scale;
    const auto cell = ix.cells.find(TreeIndex::key(cx, cy, cz));
    if (cell == ix.cells.end()) return false;
    const float S = float(scale), iS = 1.0f / S;
    const float ox = float(cx * 32), oy = float(cy * 32), oz = float(cz * 32);   // в вокселях размера scale
    std::fill(out, out + 32 * 32 * 32, AIR);
    if (branchOut) std::fill(branchOut, branchOut + 32 * 32 * 32, uint16_t(0xFFFF));
    bool any = false;
    const auto idx = [](int x, int y, int z) { return (size_t(y) * 32 + size_t(z)) * 32 + size_t(x); };
    const uint32_t ns = t.seed * 0x27D4EB2Du + 0x165667B1u;
    if (wood)
        for (const int si : cell->second.segs) {
            const size_t i = size_t(si);
            const TreeNode& nd = t.nodes[i];
            const TreeNode& pa = t.nodes[size_t(nd.parent)];
            const float ra = pa.r * iS, rb = nd.r * iS;
            if (std::max(ra, rb) < 0.3f && t.branches[size_t(nd.branch)].order > 0) continue;   // тоньше трети вокселя
            const V3 a = V3{pa.p[0], pa.p[1], pa.p[2]} * iS - V3{ox, oy, oz}, b = V3{nd.p[0], nd.p[1], nd.p[2]} * iS - V3{ox, oy, oz};
            const float rmax = std::max(ra, rb) * 1.1f + 1;
            const int x0 = std::max(0, int(std::floor(std::min(a.x, b.x) - rmax))), x1 = std::min(31, int(std::ceil(std::max(a.x, b.x) + rmax)));
            const int y0 = std::max(0, int(std::floor(std::min(a.y, b.y) - rmax))), y1 = std::min(31, int(std::ceil(std::max(a.y, b.y) + rmax)));
            const int z0 = std::max(0, int(std::floor(std::min(a.z, b.z) - rmax))), z1 = std::min(31, int(std::ceil(std::max(a.z, b.z) + rmax)));
            if (x0 > x1 || y0 > y1 || z0 > z1) continue;
            const V3 ab = b - a;
            const float l2 = std::max(1e-6f, dot(ab, ab));
            const bool dead = t.branches[size_t(nd.branch)].dead;
            for (int y = y0; y <= y1; ++y)
                for (int z = z0; z <= z1; ++z)
                    for (int x = x0; x <= x1; ++x) {
                        const V3 q{x + 0.5f, y + 0.5f, z + 0.5f};
                        const float tt = std::clamp(dot(q - a, ab) / l2, 0.0f, 1.0f);
                        float rr = ra + (rb - ra) * tt;
                        if (rr * S > 3.0f) rr *= 1.0f + 0.05f * noise3((q + V3{ox, oy, oz}) * (S / 24.0f), ns);
                        const float dd = len(q - (a + ab * tt));
                        if (dd >= std::max(rr, 0.75f)) continue;
                        const size_t k = idx(x, y, z);
                        const bool inner = dd < rr - 1.6f * iS - 0.5f && rr > 1.5f;
                        const Material m = inner ? WOOD : (dead ? BARK2 : BARK);
                        if (out[k] == AIR || out[k] == LEAVES || out[k] == LEAVES2 || (m == WOOD && out[k] != WOOD)) {
                            // древесина внутри побеждает кору соседнего отрезка - на стыках нет корки внутри
                            if (!(out[k] == WOOD && m != WOOD)) {
                                out[k] = m;
                                if (branchOut) branchOut[k] = uint16_t(nd.branch);
                            }
                        }
                        any = true;
                    }
        }
    if (leaves)
        for (const int li : cell->second.leaves) {
            const TreeLeaf& lf = t.leaves[size_t(li)];
            const V3 c = V3{lf.c[0], lf.c[1], lf.c[2]} * iS - V3{ox, oy, oz};
            const float r = lf.r * iS, rm = r * 1.2f;
            const int x0 = std::max(0, int(std::floor(c.x - rm))), x1 = std::min(31, int(std::ceil(c.x + rm)));
            const int y0 = std::max(0, int(std::floor(c.y - rm))), y1 = std::min(31, int(std::ceil(c.y + rm)));
            const int z0 = std::max(0, int(std::floor(c.z - rm))), z1 = std::min(31, int(std::ceil(c.z + rm)));
            if (x0 > x1 || y0 > y1 || z0 > z1) continue;
            for (int y = y0; y <= y1; ++y)
                for (int z = z0; z <= z1; ++z)
                    for (int x = x0; x <= x1; ++x) {
                        const V3 q{x + 0.5f, y + 0.5f, z + 0.5f};
                        const V3 w = q + V3{ox, oy, oz};
                        const float d = len(q - c);
                        // комок сплюснут сверху вниз: дубовый лист лежит пластами
                        const V3 e = q - c;
                        const float df = std::sqrt(e.x * e.x + e.z * e.z + 2.2f * e.y * e.y);
                        if (df > r * (0.8f + 0.35f * noise3(w * (0.25f * S), ns ^ 0xA5A5u))) continue;
                        if (scale <= 2 && noise3(w * (0.6f * S), ns ^ 0x5A5Au) > 0.30f) continue;   // просветы
                        (void)d;
                        const size_t k = idx(x, y, z);
                        if (out[k] != AIR) continue;
                        out[k] = noise3(w * (0.15f * S), ns ^ 0x3C3Cu) > 0.1f ? LEAVES2 : LEAVES;
                        if (branchOut) branchOut[k] = uint16_t(lf.branch);
                        any = true;
                    }
        }
    return any;
}

}  // namespace world
