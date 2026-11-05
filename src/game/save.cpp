#include "game/save.h"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <type_traits>

namespace game {
namespace {

constexpr uint32_t MAGIC = 0x4C4D544Fu;   // "OTML"
constexpr uint32_t VERSION = 5;   // Р107: срезанные растения, возраст предмета

struct W {
    std::vector<uint8_t> b;
    template <class T> void pod(const T& v) {
        static_assert(std::is_trivially_copyable_v<T>);
        const auto* p = reinterpret_cast<const uint8_t*>(&v);
        b.insert(b.end(), p, p + sizeof(T));
    }
    template <class T> void vec(const std::vector<T>& v) {
        pod(uint32_t(v.size()));
        for (const T& x : v) pod(x);
    }
};
struct R {
    const std::vector<uint8_t>& b;
    size_t at = 0;
    bool ok = true;
    template <class T> void pod(T& v) {
        static_assert(std::is_trivially_copyable_v<T>);
        if (at + sizeof(T) > b.size()) { ok = false; return; }
        std::memcpy(&v, b.data() + at, sizeof(T));
        at += sizeof(T);
    }
    template <class T> void vec(std::vector<T>& v) {
        uint32_t n = 0;
        pod(n);
        if (!ok || n > (1u << 28)) { ok = false; return; }
        v.resize(n);
        for (T& x : v) pod(x);
    }
};

// ─── модель предмета: воксели сжаты отрезками одинакового материала
void putModel(W& w, const ItemModel& m) {
    w.pod(m.sx); w.pod(m.sy); w.pod(m.sz); w.pod(m.res); w.pod(m.grip);
    uint32_t runs = 0;
    const size_t at = w.b.size();
    w.pod(runs);
    for (size_t i = 0; i < m.v.size();) {
        size_t j = i;
        while (j < m.v.size() && m.v[j] == m.v[i] && j - i < 65535) ++j;
        w.pod(uint16_t(j - i));
        w.pod(m.v[i]);
        ++runs;
        i = j;
    }
    std::memcpy(w.b.data() + at, &runs, sizeof(runs));
}
std::shared_ptr<ItemModel> getModel(R& r) {
    auto m = std::make_shared<ItemModel>();
    r.pod(m->sx); r.pod(m->sy); r.pod(m->sz); r.pod(m->res); r.pod(m->grip);
    uint32_t runs = 0;
    r.pod(runs);
    if (!r.ok || m->sx <= 0 || m->sy <= 0 || m->sz <= 0 || size_t(m->sx) * m->sy * m->sz > (64u << 20)) { r.ok = false; return m; }
    m->v.reserve(size_t(m->sx) * m->sy * m->sz);
    for (uint32_t k = 0; k < runs && r.ok; ++k) {
        uint16_t n = 0;
        world::Material mat = world::AIR;
        r.pod(n);
        r.pod(mat);
        m->v.insert(m->v.end(), n, mat);
    }
    if (m->v.size() != size_t(m->sx) * m->sy * m->sz) { r.ok = false; return m; }
    m->finish();
    return m;
}
void putItem(W& w, const Item& it) {
    w.pod(it.kind);
    w.pod(it.size);
    w.pod(it.mat);
    putModel(w, *it.model);
}
Item getItem(R& r) {
    Item it;
    r.pod(it.kind);
    r.pod(it.size);
    r.pod(it.mat);
    it.model = getModel(r);
    return it;
}

void putSkel(W& w, const world::TreeSkel& t) {
    w.pod(t.age); w.pod(t.species); w.pod(t.seed);
    w.vec(t.nodes); w.vec(t.branches); w.vec(t.leaves);
    w.pod(t.lo); w.pod(t.hi); w.pod(t.height); w.pod(t.crownR); w.pod(t.roots);
}
std::shared_ptr<world::TreeSkel> getSkel(R& r) {
    auto t = std::make_shared<world::TreeSkel>();
    r.pod(t->age); r.pod(t->species); r.pod(t->seed);
    r.vec(t->nodes); r.vec(t->branches); r.vec(t->leaves);
    r.pod(t->lo); r.pod(t->hi); r.pod(t->height); r.pod(t->crownR); r.pod(t->roots);
    return t;
}

void putCut(W& w, const TreeCut& c) {
    w.pod(c.notched); w.pod(c.felled); w.pod(c.y0); w.pod(c.dir); w.pod(c.center); w.pod(c.R); w.pod(c.depth); w.pod(c.hits);
    w.vec(c.part); w.pod(c.rootsCut); w.vec(c.rootOf); w.pod(c.roots); w.pod(c.rootTarget); w.pod(c.rootHits); w.pod(c.dug); w.pod(c.digHits);
    w.pod(c.leafKeep); w.pod(c.scraped);
}
TreeCut getCut(R& r) {
    TreeCut c;
    r.pod(c.notched); r.pod(c.felled); r.pod(c.y0); r.pod(c.dir); r.pod(c.center); r.pod(c.R); r.pod(c.depth); r.pod(c.hits);
    r.vec(c.part); r.pod(c.rootsCut); r.vec(c.rootOf); r.pod(c.roots); r.pod(c.rootTarget); r.pod(c.rootHits); r.pod(c.dug); r.pod(c.digHits);
    r.pod(c.leafKeep); r.pod(c.scraped);
    return c;
}

void putBody(W& w, const Body& b) { w.pod(b.pos); w.pod(b.rot); w.pod(b.vel); w.pod(b.ang); w.pod(b.asleep); w.pod(b.still); }
Body getBody(R& r) { Body b; r.pod(b.pos); r.pod(b.rot); r.pod(b.vel); r.pod(b.ang); r.pod(b.asleep); r.pod(b.still); return b; }

}  // namespace

bool saveWorld(const std::string& path, const SaveData& d) {
    W w;
    w.pod(MAGIC); w.pod(VERSION); w.pod(d.seed);
    w.pod(d.eye); w.pod(d.yaw); w.pod(d.pitch); w.pod(d.flying);
    w.vec(d.edits);
    for (const auto& h : d.hands) {
        w.pod(uint8_t(h.has_value()));
        if (h) putItem(w, *h);
    }
    w.pod(uint32_t(d.cuts.size()));
    for (const auto& [key, c] : d.cuts) { w.pod(key); putCut(w, c); }
    w.pod(uint32_t(d.bodies.size()));
    for (const TreeBody& tb : d.bodies) {
        w.pod(tb.key); w.pod(tb.base); w.pod(tb.shift);
        putSkel(w, *tb.skel);
        putCut(w, tb.cut);
        w.pod(tb.free);
        w.pod(tb.hinge.hinge); w.pod(tb.hinge.axis); w.pod(tb.hinge.theta); w.pod(tb.hinge.omega); w.pod(tb.hinge.I); w.pod(tb.hinge.com);
        w.vec(tb.hinge.probes);
        putBody(w, tb.body);
        w.pod(tb.mass); w.pod(tb.inertia); w.vec(tb.probes); w.pod(tb.com); w.pod(tb.crushed);
        w.vec(tb.removed); w.pod(tb.yLo); w.pod(tb.yHi); w.vec(tb.attachY); w.vec(tb.debarked);
    }
    w.pod(uint32_t(d.items.size()));
    for (const WorldItem& it : d.items) {
        putItem(w, it.item);
        putBody(w, it.body);
        w.pod(it.placed);
        w.pod(it.age);
    }
    w.vec(d.taken);
    w.vec(d.plantsCut);
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
    const std::string tmp = path + ".tmp";
    FILE* f = std::fopen(tmp.c_str(), "wb");
    if (!f) return false;
    const bool ok = std::fwrite(w.b.data(), 1, w.b.size(), f) == w.b.size();
    std::fclose(f);
    if (!ok) return false;
    std::filesystem::rename(tmp, path, ec);
    return !ec;
}

bool loadWorld(const std::string& path, SaveData& d) {
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    std::vector<uint8_t> buf;
    uint8_t tmp[65536];
    size_t n;
    while ((n = std::fread(tmp, 1, sizeof(tmp), f)) > 0) buf.insert(buf.end(), tmp, tmp + n);
    std::fclose(f);
    R r{buf};
    uint32_t magic = 0, ver = 0;
    r.pod(magic); r.pod(ver);
    if (!r.ok || magic != MAGIC || ver != VERSION) return false;
    r.pod(d.seed);
    r.pod(d.eye); r.pod(d.yaw); r.pod(d.pitch); r.pod(d.flying);
    r.vec(d.edits);
    for (auto& h : d.hands) {
        uint8_t has = 0;
        r.pod(has);
        if (has) h = getItem(r);
    }
    uint32_t nc = 0;
    r.pod(nc);
    for (uint32_t i = 0; i < nc && r.ok; ++i) {
        uint64_t key = 0;
        r.pod(key);
        d.cuts.push_back({key, getCut(r)});
    }
    uint32_t nb = 0;
    r.pod(nb);
    for (uint32_t i = 0; i < nb && r.ok; ++i) {
        TreeBody tb;
        r.pod(tb.key); r.pod(tb.base); r.pod(tb.shift);
        tb.skel = getSkel(r);
        tb.cut = getCut(r);
        r.pod(tb.free);
        r.pod(tb.hinge.hinge); r.pod(tb.hinge.axis); r.pod(tb.hinge.theta); r.pod(tb.hinge.omega); r.pod(tb.hinge.I); r.pod(tb.hinge.com);
        r.vec(tb.hinge.probes);
        tb.body = getBody(r);
        r.pod(tb.mass); r.pod(tb.inertia); r.vec(tb.probes); r.pod(tb.com); r.pod(tb.crushed);
        r.vec(tb.removed); r.pod(tb.yLo); r.pod(tb.yHi); r.vec(tb.attachY); r.vec(tb.debarked);
        tb.meshDirty = true;
        d.bodies.push_back(std::move(tb));
    }
    uint32_t ni = 0;
    r.pod(ni);
    for (uint32_t i = 0; i < ni && r.ok; ++i) {
        WorldItem it;
        it.item = getItem(r);
        it.body = getBody(r);
        r.pod(it.placed);
        r.pod(it.age);
        d.items.push_back(std::move(it));
    }
    r.vec(d.taken);
    r.vec(d.plantsCut);
    return r.ok;
}

}  // namespace game
