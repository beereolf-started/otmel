// Р77: породы в разрезе - первый шаг Р76. Набор - 20 пород TFC/TFG (импорт схематик из TFG один к одному), граф
// слоёв - как у TFC (docs/data/tfg/rocks.json): у каждой породы слоя - слой под ней, цепочки сходятся к нижнему
// ярусу. Тип района - из наших полей (Р76 п.3): океан - доля суши, вулканизм - новое медленное поле, горы - вес гор.
// Отличия от TFC (Р76 п.2):
//   - границы слоёв - от поверхности ДО размыва (крупный рельеф без октав мельче 2048 блоков): долины и обрывы
//     режут пласты, а не повторяют их;
//   - в горах пласты смяты: границы сдвигаются шумом, сдвиг ~ вес гор (складки);
//   - переходы плавные (Р76 п.1): у границы слоёв - полоса ±6 блоков, где блоки двух пород перемешаны по весу; на
//     стыке районов одного слоя - так же по горизонтали. Резко - только под излившимися (лава легла, не смешавшись)
//     и между пластами осадков (слоистость: пласты по ~7 блоков, порода чередуется - горы полосатые).
#include <algorithm>

#include "gen/hash.h"
#include "gen/noise.h"
#include "world/terrain.h"

namespace world {
namespace {

enum Layer : int8_t { FELSIC, INTERMEDIATE, MAFIC, EXTRUSIVE, EXTRUSIVE2, PHYLLITE_L, SLATE_L, MARBLE_L, QUARTZITE_L,
                      SEDIMENTARY, UPLIFT, BOTTOM };
struct LayerDef { int n; Rock rock[7]; Layer next[7]; bool extrusive, sediment; };
// граф TFC: порода слоя -> слой под ней (world_preset/overworld.json, rock_layer_settings)
const LayerDef LAYERS[] = {
    {1, {GRANITE}, {BOTTOM}, false, false},                                                          // felsic
    {1, {DIORITE}, {BOTTOM}, false, false},                                                          // intermediate
    {1, {GABBRO}, {BOTTOM}, false, false},                                                           // mafic
    {4, {RHYOLITE, ANDESITE, DACITE, BASALT}, {FELSIC, INTERMEDIATE, INTERMEDIATE, MAFIC}, true, false},
    {4, {RHYOLITE, ANDESITE, DACITE, BASALT}, {EXTRUSIVE, EXTRUSIVE, EXTRUSIVE, EXTRUSIVE}, true, false},
    {3, {PHYLLITE, GNEISS, SCHIST}, {BOTTOM, BOTTOM, BOTTOM}, false, false},
    {2, {SLATE, PHYLLITE}, {BOTTOM, PHYLLITE_L}, false, false},
    {1, {MARBLE}, {BOTTOM}, false, false},
    {1, {QUARTZITE}, {BOTTOM}, false, false},
    {7, {SHALE, CLAYSTONE, CONGLOMERATE, LIMESTONE, DOLOMITE, CHALK, CHERT},
        {SLATE_L, SLATE_L, SLATE_L, MARBLE_L, MARBLE_L, MARBLE_L, QUARTZITE_L}, false, true},
    {6, {SLATE, MARBLE, QUARTZITE, DIORITE, GRANITE, GABBRO},
        {PHYLLITE_L, BOTTOM, BOTTOM, SEDIMENTARY, SEDIMENTARY, SEDIMENTARY}, false, false},
    {5, {GNEISS, SCHIST, DIORITE, GRANITE, GABBRO}, {BOTTOM, BOTTOM, BOTTOM, BOTTOM, BOTTOM}, false, false},
};
constexpr uint32_t ROCK_SEED = 0x50C4u;
constexpr int32_t BAND = 6;           // полуширина перехода между слоями, блоки
constexpr int32_t BED = 7;            // толщина пласта осадков, блоки
constexpr int32_t THICK = 100, THICK_VAR = 40;   // толщина слоя: 60..140 блоков

float unit(fx::q16 v) { return float(v) / float(fx::ONE); }   // [-1, 1]

}  // namespace

void rockColumn(const TerrainParams& p, int32_t x, int32_t z, const Relief& r, RockColumn& out) {
    const uint32_t s = p.seed ^ ROCK_SEED;
    const float land = unit(r.land), mount = unit(r.wMount) * land;
    const float volc = unit(gen::value2(x, z, 13, gen::hash2(1, 0, s)));
    // тип района -> с каких слоёв начинать (как у TFC: дно океана, вулканический, суша, поднятие)
    Layer start[2];
    if (land < 0.5f) start[0] = start[1] = EXTRUSIVE;
    else if (volc > 0.55f) { start[0] = EXTRUSIVE; start[1] = EXTRUSIVE2; }
    else if (mount > 0.45f) { start[0] = SEDIMENTARY; start[1] = UPLIFT; }
    else { start[0] = EXTRUSIVE; start[1] = SEDIMENTARY; }
    Layer layer = start[unit(gen::value2(x, z, 12, gen::hash2(2, 0, s))) > 0.0f ? 1 : 0];
    // поверхность до размыва и складки
    const int32_t ref = baseReliefAt(p, x, z, slowAt(p, x, z, 11), 11).height;
    out.fold = int32_t(mount * 120.0f * unit(gen::simplex2(x, z, 9, gen::hash2(3, 0, s))) +
                       25.0f * unit(gen::simplex2(x, z, 11, gen::hash2(4, 0, s))));
    int32_t bottom = ref + out.fold;
    out.sharp = out.bedded = 0;
    for (int i = 0; i < ROCK_LAYERS; ++i) {
        const LayerDef& d = LAYERS[layer];
        // какая порода слоя - медленный шум (районы ~4 км); у стыка районов - вторая порода по весу
        const float v = (unit(gen::value2(x, z, 12, gen::hash2(10 + i, layer, s))) + 1.0f) * 0.5f * float(d.n);
        const int id = std::clamp(int(v), 0, d.n - 1);
        const float frac = v - float(id), edge = std::min(frac, 1.0f - frac);
        int alt = frac < 0.5f ? id - 1 : id + 1;
        alt = (alt + d.n) % d.n;
        float altW = edge < 0.12f ? 0.5f * (1.0f - edge / 0.12f) : 0.0f;
        if (d.sediment) {   // пласты: вторая порода чередуется с основной
            alt = (id + 1 + int(gen::hash2(20 + i, layer, s) % uint32_t(d.n - 1))) % d.n;
            altW = 0.38f;
            out.bedded |= uint8_t(1u << i);
        }
        out.id[i] = d.rock[id];
        out.alt[i] = d.rock[alt];
        out.altW[i] = uint8_t(altW * 255.0f);
        if (i < ROCK_LAYERS - 1) {
            const float t = unit(gen::value2(x, z, 11, gen::hash2(30 + i, layer, s)));
            // толщина пропорциональна рельефу (Р76 п.6): в горах долины режут на километры - слои вчетверо толще
            bottom -= int32_t((float(THICK) + float(THICK_VAR) * t) * (1.0f + 4.0f * mount));
            out.bound[i] = bottom;
            if (d.extrusive) out.sharp |= uint8_t(1u << i);   // под лавой - резко
        }
        if (layer != BOTTOM) layer = d.next[id];
    }
}

Material rockAt(const RockColumn& c, int32_t x, int32_t y, int32_t z) {
    int i = 0;
    while (i < ROCK_LAYERS - 1 && y <= c.bound[i]) ++i;
    const uint32_t h = gen::hash3(x, y, z, ROCK_SEED);
    const float r0 = float(h & 0xFFFF) / 65536.0f, r1 = float(h >> 16) / 65536.0f;
    // переход между слоями: у верхней границы - доля породы сверху, у нижней - снизу (до 50% на самой границе)
    int j = i;
    if (i > 0 && !(c.sharp >> (i - 1) & 1)) {
        const int32_t d = c.bound[i - 1] - y;
        if (d < BAND && r0 < 0.5f * (1.0f - float(d) / BAND)) j = i - 1;
    }
    if (j == i && i < ROCK_LAYERS - 1 && !(c.sharp >> i & 1)) {
        const int32_t d = y - c.bound[i];
        if (d < BAND && r0 < 0.5f * (1.0f - float(d) / BAND)) j = i + 1;
    }
    i = j;
    float altP = float(c.altW[i]) / 255.0f;
    if (c.bedded >> i & 1) {   // пласт целиком одной породы: решает номер пласта (сквозной по миру, смят складками)
        const int32_t bed = (y - c.fold) >= 0 ? (y - c.fold) / BED : -((c.fold - y + BED - 1) / BED);
        return rockMaterial(Rock(float(gen::hash2(bed, i, ROCK_SEED) & 0xFFFF) / 65536.0f < altP ? c.alt[i] : c.id[i]));
    }
    return rockMaterial(Rock(r1 < altP ? c.alt[i] : c.id[i]));
}

}  // namespace world
