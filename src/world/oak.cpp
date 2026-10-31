#include "world/oak.h"

#include <algorithm>
#include <cmath>

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

// случайные числа от семени - одно семя, одно дерево
struct Rng {
    uint64_t s;
    explicit Rng(uint32_t seed) : s(0x9E3779B97F4A7C15ull ^ (uint64_t(seed) * 0xD1B54A32D192ED03ull)) { next(); }
    uint32_t next() {
        s += 0x9E3779B97F4A7C15ull;
        uint64_t z = s;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return uint32_t((z ^ (z >> 31)) >> 32);
    }
    float f() { return float(next() >> 8) / 16777216.0f; }             // 0..1
    float r(float a, float b) { return a + (b - a) * f(); }
    V3 unit() {
        const float z = r(-1, 1), a = r(0, 6.2831853f), q = std::sqrt(1 - z * z);
        return {q * std::cos(a), z, q * std::sin(a)};
    }
};

// 3D-шум значений (трилинейно, плавно), -1..1 - наплывы коры и форма листвы
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

struct Node { V3 p; int parent; int order; bool root, stub; float r = 0; int children = 0; };

struct Grower {
    Rng rng;
    std::vector<Node> nodes;
    float golden = 0;
    explicit Grower(uint32_t seed) : rng(seed) {}

    int add(V3 p, int parent, int order, bool root, bool stub) {
        nodes.push_back({p, parent, order, root, stub});
        if (parent >= 0) ++nodes[size_t(parent)].children;
        return int(nodes.size()) - 1;
    }

    // направление вбок от d под углом a: вокруг d по золотому углу - соседние отводы не в одну сторону
    V3 aside(V3 d, float a) {
        const V3 ref = std::fabs(d.y) < 0.9f ? UP : V3{1, 0, 0};
        const V3 u = norm(cross(d, ref)), w = cross(d, u);
        golden += 2.39996f + rng.r(-0.3f, 0.3f);
        const V3 side = u * std::cos(golden) + w * std::sin(golden);
        return norm(d * std::cos(a) + side * std::sin(a));
    }

    // ветвь от узла from: шагами, направление блуждает (κ), тянется вверх (φ), длинная провисает (σ)
    void branch(int from, V3 dir, float length, int order, bool root, bool stub = false) {
        static constexpr float STEP[5] = {6, 5, 4, 3, 3};
        static constexpr float KAPPA[5] = {0.12f, 0.32f, 0.42f, 0.55f, 0.55f};
        static constexpr float PHI[5] = {0.45f, 0.3f, 0.24f, 0.2f, 0.2f};
        static constexpr float SAG[5] = {0, 0.06f, 0.05f, 0.03f, 0.03f};
        static constexpr float FORK[5] = {0, 0.65f, 0.7f, 0, 0};       // отвод на шаг (Р61: чаще - крона пышнее)
        const float step = STEP[std::min(order, 4)];
        const int n = std::max(1, int(length / step + 0.5f));
        V3 w = norm(cross(dir, rng.unit()));
        int at = from;
        V3 p = nodes[size_t(from)].p;
        for (int i = 1; i <= n; ++i) {
            w = norm(w * 0.72f + rng.unit() * 0.5f);
            const float t = float(i) / n;
            const float kappa = KAPPA[std::min(order, 4)] * (root ? 0.8f : 1.0f);
            const float phi = root ? -0.22f : PHI[std::min(order, 4)];
            const float sag = root ? 0 : SAG[std::min(order, 4)] * t;
            dir = norm(dir + w * kappa + UP * (phi - sag));
            if (root && dir.y > -0.05f) dir = norm(dir + UP * -0.2f);   // корень не лезет из земли вверх
            p = p + dir * step;
            at = add(p, at, order, root, stub);
            if (stub) continue;
            // отводы: не у самого начала ветви; у корней - реже
            const int maxOrder = root ? 2 : 3;
            if (order >= 1 && order < maxOrder && t > 0.18f && rng.f() < FORK[order] * (root ? 0.5f : 1.0f)) {
                const float a = rng.r(0.75f, 1.3f);   // 43..75°
                const V3 cd = aside(dir, a);
                if (!root && rng.f() < 0.12f)
                    branch(at, cd, rng.r(4, 9), order + 1, false, true);   // обломанный сучок
                else if (root)
                    branch(at, cd, length * (1 - t) * rng.r(0.4f, 0.7f), order + 1, true);
                else
                    branch(at, norm(cd + UP * (order == 1 ? 0.3f : 0.15f)),
                           order == 1 ? rng.r(22, 38) : rng.r(8, 14), order + 1, false);
            }
        }
    }

    void grow() {
        // ствол: низкий, с лёгким наклоном; комель - узел 0
        const int base = add({0, 0, 0}, -1, 0, false, false);
        const float bole = rng.r(104, 124);   // 6.5-7.8 блока до развилки (Р57: ствол выше на 3-4 блока)
        V3 d = norm(UP + V3{rng.r(-0.07f, 0.07f), 0, rng.r(-0.07f, 0.07f)});   // Р71: стройнее
        const int n = int(bole / 6);
        int at = base;
        V3 p{0, 0, 0};
        V3 w = norm(cross(d, rng.unit()));
        for (int i = 0; i < n; ++i) {
            w = norm(w * 0.7f + rng.unit() * 0.5f);
            d = norm(d + w * 0.035f + UP * 0.45f);   // ствол - прямой столб (удобно рубить), Р71: почти без блуждания
            p = p + d * 6;
            at = add(p, at, 0, false, false);
            if (i > 1 && i < n - 1 && rng.f() < 0.1f) branch(at, aside(d, 1.2f), rng.r(4, 8), 1, false, true);
        }
        // развилка: 2-4 сука врозь от вертикали
        const int limbs = 2 + int(rng.f() * 2);
        const float az0 = rng.r(0, 6.283f);
        for (int k = 0; k < limbs; ++k) {
            const float az = az0 + 6.283f * k / limbs + rng.r(-0.4f, 0.4f), a = rng.r(0.35f, 0.7f);
            const V3 ld = norm(UP * std::cos(a) + V3{std::cos(az), 0, std::sin(az)} * std::sin(a));
            branch(at, ld, rng.r(84, 112), 1, false);
        }
        // корни: от комля наружу и вниз
        const int roots = 4 + int(rng.f() * 3);
        const int collar = std::min<int>(1, int(nodes.size()) - 1);
        for (int k = 0; k < roots; ++k) {
            const float az = 6.283f * k / roots + rng.r(-0.35f, 0.35f), b = rng.r(0.26f, 0.6f);
            const V3 rd = norm(V3{std::cos(az), 0, std::sin(az)} * std::cos(b) + UP * -std::sin(b));
            branch(collar == 1 ? 1 : 0, rd, rng.r(16, 34), 1, true);
        }
    }

    // толщина: правило Леонардо, от кончиков к комлю (дети всегда после родителя); комель расширен
    void radii() {
        constexpr float E = 2.5f, TIP = 0.85f, ROOT_TIP = 1.0f;   // Р57: молодой дуб - тоньше
        std::vector<float> acc(nodes.size(), 0.0f);
        for (size_t i = nodes.size(); i-- > 0;) {
            Node& nd = nodes[i];
            const float tip = nd.root ? ROOT_TIP : (nd.stub ? 1.6f : TIP);
            nd.r = nd.children == 0 ? tip : std::max(tip, std::pow(acc[i], 1.0f / E));
            if (nd.parent >= 0) acc[size_t(nd.parent)] += std::pow(nd.r, E);
        }
        // Р72: сбег ствола - от комля к развилке сужается (у комля на четверть толще), плюс корневой наплыв у земли
        float bole = 1.0f;
        for (const Node& nd : nodes) if (!nd.root && nd.order == 0) bole = std::max(bole, nd.p.y);
        for (Node& nd : nodes)
            if (!nd.root && nd.order == 0)
                nd.r *= (1.0f + 0.25f * (1.0f - std::clamp(nd.p.y / bole, 0.0f, 1.0f))) *
                        (1.0f + 0.3f * std::exp(-std::max(0.0f, nd.p.y) / 5.0f));
    }
};

}  // namespace

void growOak(uint32_t seed, OakVoxels& out, int scale) {
    Grower g(seed);
    g.grow();
    g.radii();
    // Р63: всё дальше - в вокселях размера scale (мелких): точки и радиусы делятся на scale
    const float S = float(scale), iS = 1.0f / S;
    out.scale = scale;
    for (Node& nd : g.nodes) { nd.p = nd.p * iS; nd.r *= iS; }
    const uint32_t ns = seed * 0x27D4EB2Du + 0x165667B1u;
    // листва: комки у кончиков веточек (порядок 2-3, не пеньки, не корни)
    struct Blob { V3 c; float r; };
    std::vector<Blob> blobs;
    for (const Node& nd : g.nodes)
        if (!nd.root && !nd.stub && nd.order >= 2 && (nd.children == 0 || g.rng.f() < 0.5f))
            blobs.push_back({nd.p + (UP * g.rng.r(1, 3) + g.rng.unit() * 1.5f) * iS, (nd.children == 0 ? g.rng.r(11, 16) : g.rng.r(9, 13)) * iS});
    // рамка
    float lo[3] = {1e9f, 1e9f, 1e9f}, hi[3] = {-1e9f, -1e9f, -1e9f};
    const auto grow = [&](V3 p, float r) {
        lo[0] = std::min(lo[0], p.x - r); lo[1] = std::min(lo[1], p.y - r); lo[2] = std::min(lo[2], p.z - r);
        hi[0] = std::max(hi[0], p.x + r); hi[1] = std::max(hi[1], p.y + r); hi[2] = std::max(hi[2], p.z + r);
    };
    for (const Node& nd : g.nodes) grow(nd.p, nd.r * 1.3f + 2);
    for (const Blob& b : blobs) grow(b.c, b.r * 1.3f + 1);
    out.x0 = int(std::floor(lo[0])); out.y0 = int(std::floor(lo[1])); out.z0 = int(std::floor(lo[2]));
    out.sx = int(std::ceil(hi[0])) - out.x0 + 1; out.sy = int(std::ceil(hi[1])) - out.y0 + 1; out.sz = int(std::ceil(hi[2])) - out.z0 + 1;
    out.v.assign(size_t(out.sx) * out.sy * out.sz, AIR);
    out.root.assign(out.v.size(), 0);
    out.nodes = int(g.nodes.size());
    const auto idx = [&](int x, int y, int z) { return (size_t(y - out.y0) * out.sz + size_t(z - out.z0)) * out.sx + size_t(x - out.x0); };
    // ветви: усечённые конусы родитель -> узел; радиус гуляет шумом (наплывы); снаружи кора, внутри древесина
    for (const Node& nd : g.nodes) {
        if (nd.parent < 0) continue;
        const Node& pa = g.nodes[size_t(nd.parent)];
        if (nd.order > 0 && std::max(pa.r, nd.r) < 0.3f) continue;   // Р63: тоньше трети вокселя - не видно
        const V3 a = pa.p, b = nd.p, ab = b - a;
        const float l2 = std::max(1e-6f, dot(ab, ab));
        const float rmax = std::max(pa.r, nd.r) * 1.25f + 1;
        const int x0 = int(std::floor(std::min(a.x, b.x) - rmax)), x1 = int(std::ceil(std::max(a.x, b.x) + rmax));
        const int y0 = int(std::floor(std::min(a.y, b.y) - rmax)), y1 = int(std::ceil(std::max(a.y, b.y) + rmax));
        const int z0 = int(std::floor(std::min(a.z, b.z) - rmax)), z1 = int(std::ceil(std::max(a.z, b.z) + rmax));
        for (int y = y0; y <= y1; ++y)
            for (int z = z0; z <= z1; ++z)
                for (int x = x0; x <= x1; ++x) {
                    const V3 q{x + 0.5f, y + 0.5f, z + 0.5f};
                    const float t = std::clamp(dot(q - a, ab) / l2, 0.0f, 1.0f);
                    const V3 f = a + ab * t;
                    float rr = pa.r + (nd.r - pa.r) * t;
                    // Р72: наплывы - пологие и редкие (±6%, ~30 мелких вокселей): частые ±22% делали ствол свитым
                    // канатом. Рисунок коры - потом текстурой, не объёмом.
                    if (rr > 2.0f) rr *= 1.0f + 0.06f * noise3(q * (S / 30.0f), ns);
                    const float dd = len(q - f);
                    if (dd >= std::max(rr, 0.75f)) continue;   // тоньше вокселя - линия без разрывов
                    const size_t k = idx(x, y, z);
                    const bool inner = dd < rr - 1.6f && rr > 2.2f;
                    if (inner) out.v[k] = WOOD;
                    else if (out.v[k] == AIR || out.v[k] == LEAVES || out.v[k] == LEAVES2) out.v[k] = BARK;
                    if (nd.root) out.root[k] = 1;
                }
    }
    // листва: шар, край гуляет шумом, внутри просветы
    for (const Blob& bl : blobs) {
        const float rm = bl.r * 1.2f;
        for (int y = int(std::floor(bl.c.y - rm)); y <= int(std::ceil(bl.c.y + rm)); ++y)
            for (int z = int(std::floor(bl.c.z - rm)); z <= int(std::ceil(bl.c.z + rm)); ++z)
                for (int x = int(std::floor(bl.c.x - rm)); x <= int(std::ceil(bl.c.x + rm)); ++x) {
                    const V3 q{x + 0.5f, y + 0.5f, z + 0.5f};
                    const float d = len(q - bl.c);
                    if (d > bl.r * (0.8f + 0.35f * noise3(q * (0.25f * S), ns ^ 0xA5A5u))) continue;
                    if (scale <= 2 && noise3(q * (0.6f * S), ns ^ 0x5A5Au) > 0.38f) continue;   // просветы (вдали - сплошь)
                    const size_t k = idx(x, y, z);
                    // два оттенка листвы пятнами
                    if (out.v[k] == AIR) out.v[k] = noise3(q * (0.15f * S), ns ^ 0x3C3Cu) > 0.1f ? LEAVES2 : LEAVES;
                }
    }
    for (size_t k = 0; k < out.v.size(); ++k) {
        if (out.v[k] != AIR) out.top = std::max(out.top, (out.y0 + int(k / (size_t(out.sx) * out.sz)) + 1) * scale);
        if (out.v[k] == LEAVES || out.v[k] == LEAVES2) ++out.leaves;   // кора в трещине (BARK2) - древесина
        else if (out.v[k] != AIR) { ++out.wood; if (out.root[k]) ++out.roots; }
    }
}

}  // namespace world
