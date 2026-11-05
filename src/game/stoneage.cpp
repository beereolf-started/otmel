#include "game/stoneage.h"

#include <algorithm>
#include <cmath>

namespace game {
namespace {

uint32_t mix(uint32_t h) { h ^= h >> 15; h *= 0x2C1B3C6Du; h ^= h >> 12; h *= 0x297A2D39u; h ^= h >> 15; return h; }
float unit(uint32_t& h) { h = mix(h + 0x9E3779B9u); return float(h & 0xFFFFFF) / 16777216.0f; }

std::shared_ptr<ItemModel> blank(int sx, int sy, int sz, int res) {
    auto m = std::make_shared<ItemModel>();
    m->sx = sx; m->sy = sy; m->sz = sz; m->res = res;
    m->v.assign(size_t(sx) * sy * sz, world::AIR);
    return m;
}

// образцы тёски (TFC): X - камень остаётся
struct Pattern { ItemKind kind; const char* rows[5]; };
const Pattern PATTERNS[] = {
    {ItemKind::HEAD_AXE, {".X...", "XXXX.", "XXXXX", "XXXX.", ".X..."}},
    {ItemKind::HEAD_KNIFE, {"X....", "XX...", "XX...", "XXX..", "XXX.."}},
    {ItemKind::HEAD_SHOVEL, {".XXX.", ".XXX.", ".XXX.", ".XXX.", "..X.."}},
    {ItemKind::HEAD_SPEAR, {"XXX..", "XXXX.", "XXXXX", ".XXX.", "..X.."}},
};

}  // namespace

Item makeStone(world::Material rock, uint32_t seed) {
    uint32_t h = mix(seed);
    const float rx = 3.0f + 2.5f * unit(h), ry = 2.0f + 1.8f * unit(h), rz = 2.6f + 2.4f * unit(h);
    const int sx = int(std::ceil(rx * 2)) + 2, sy = int(std::ceil(ry * 2)) + 2, sz = int(std::ceil(rz * 2)) + 2;
    auto m = blank(sx, sy, sz, 64);
    const float p1 = unit(h) * 6.28f, p2 = unit(h) * 6.28f;
    for (int y = 0; y < sy; ++y)
        for (int z = 0; z < sz; ++z)
            for (int x = 0; x < sx; ++x) {
                const float dx = (x + 0.5f - sx * 0.5f) / rx, dy = (y + 0.5f - sy * 0.5f) / ry, dz = (z + 0.5f - sz * 0.5f) / rz;
                // окатанный, но неправильный: радиус гуляет низкой гармоникой
                const float bump = 1.0f + 0.18f * std::sin(dx * 2.3f + p1) * std::cos(dz * 2.1f + p2) + 0.08f * std::sin(dy * 3.0f + p2);
                if (dx * dx + dy * dy + dz * dz <= bump * bump) m->at(x, y, z) = rock;
            }
    m->finish();
    return Item{ItemKind::STONE, ItemSize::SMALL, m, rock};
}

Item makeStick(uint32_t seed) {
    uint32_t h = mix(seed ^ 0x5A17u);
    const int len = 10 + int(unit(h) * 7.0f);   // мелких вокселей (1/16 блока): 0.6-1 м
    auto m = blank(9, len + 2, 9, 16);
    float x = 4.5f, z = 4.5f, dx = 0, dz = 0;
    for (int y = 1; y <= len; ++y) {
        if (unit(h) < 0.3f) { dx = (unit(h) - 0.5f) * 0.9f; dz = (unit(h) - 0.5f) * 0.9f; }   // излом
        x = std::clamp(x + dx, 0.5f, 8.5f);
        z = std::clamp(z + dz, 0.5f, 8.5f);
        m->at(int(x), y, int(z)) = (y % 4 == 0) ? world::BARK2 : world::BARK;
    }
    m->finish();
    return Item{ItemKind::TWIG, ItemSize::SMALL, m, world::AIR};
}

Item makeFiber() {
    auto m = blank(10, 2, 3, 64);
    for (int x = 0; x < 10; ++x)
        for (int z = 0; z < 3; ++z) m->at(x, (x + z) & 1, z) = (x % 3) ? world::HANDLE : world::HANDLE2;
    m->finish();
    return Item{ItemKind::FIBER, ItemSize::SMALL, m, world::AIR};
}

Item makeClay() {
    auto m = blank(8, 6, 8, 64);
    const world::Material clay = world::Material(world::SOIL_FIRST + 6);   // глина (soil_table)
    for (int y = 0; y < 6; ++y)
        for (int z = 0; z < 8; ++z)
            for (int x = 0; x < 8; ++x) {
                const float dx = (x + 0.5f - 4) / 4, dy = (y + 0.5f - 3) / 3, dz = (z + 0.5f - 4) / 4;
                if (dx * dx + dy * dy + dz * dz <= 1.0f) m->at(x, y, z) = clay;
            }
    m->finish();
    return Item{ItemKind::CLAY, ItemSize::SMALL, m, clay};
}

ItemKind knapResult(const Knap& k) {
    for (const Pattern& p : PATTERNS)
        for (int mirror = 0; mirror < 2; ++mirror) {
            bool ok = true;
            for (int y = 0; y < 5 && ok; ++y)
                for (int x = 0; x < 5 && ok; ++x) {
                    const char c = p.rows[y][mirror ? 4 - x : x];
                    ok = (c == 'X') == k.cell[y][x];
                }
            if (ok) return p.kind;
        }
    return ItemKind::STONE;
}

Item makeHead(ItemKind kind, world::Material rock, const Knap& k) {
    // каждая клетка - 3×3 вокселя (1/64 блока), толщина 2; край чуть сколот
    auto m = blank(15, 15, 2, 64);
    for (int cy = 0; cy < 5; ++cy)
        for (int cx = 0; cx < 5; ++cx) {
            if (!k.cell[cy][cx]) continue;
            for (int y = 0; y < 3; ++y)
                for (int x = 0; x < 3; ++x)
                    for (int z = 0; z < 2; ++z) m->at(cx * 3 + x, (4 - cy) * 3 + y, z) = rock;
        }
    m->finish();
    return Item{kind, ItemSize::SMALL, m, rock};
}

std::shared_ptr<ItemModel> knapModel(const Knap& k) {
    auto m = blank(17, 17, 3, 64);
    for (int cy = 0; cy < 5; ++cy)
        for (int cx = 0; cx < 5; ++cx) {
            const int x0 = 1 + cx * 3, y0 = 1 + (4 - cy) * 3;
            if (k.cell[cy][cx])
                for (int y = 0; y < 3; ++y)
                    for (int x = 0; x < 3; ++x)
                        for (int z = 0; z < 2; ++z) m->at(x0 + x, y0 + y, z) = k.rock;
            if (cx == k.cx && cy == k.cy)   // курсор - рамка поверх
                for (int y = -1; y <= 3; ++y)
                    for (int x = -1; x <= 3; ++x)
                        if (x == -1 || x == 3 || y == -1 || y == 3) m->at(x0 + x, y0 + y, 2) = world::HIGHLIGHT;
        }
    m->finish();
    return m;
}

bool matchRecipe(const std::vector<const Item*>& got, Item& out, std::string& name) {
    int heads = 0, sticks = 0, fibers = 0, other = 0;
    const Item* head = nullptr;
    for (const Item* it : got) {
        if (it->kind >= ItemKind::HEAD_AXE && it->kind <= ItemKind::HEAD_SPEAR) { ++heads; head = it; }
        else if (it->stick()) ++sticks;
        else if (it->kind == ItemKind::FIBER) ++fibers;
        else ++other;
    }
    if (heads != 1 || sticks != 1 || fibers != 1 || other) return false;
    const ItemKind tool = head->kind == ItemKind::HEAD_AXE ? ItemKind::AXE
                        : head->kind == ItemKind::HEAD_KNIFE ? ItemKind::KNIFE
                        : head->kind == ItemKind::HEAD_SHOVEL ? ItemKind::SHOVEL : ItemKind::SPEAR;
    out = makeTool(tool, head->mat);
    name = "каменный " + itemName(out);
    if (tool == ItemKind::SHOVEL) name = "каменная лопата";
    if (tool == ItemKind::SPEAR) name = "каменное копьё";
    return true;
}

std::string recipesHint() {
    return "топор / нож / лопата / копьё = головка (тёска камня, E) + палка + волокно";
}

}  // namespace game
