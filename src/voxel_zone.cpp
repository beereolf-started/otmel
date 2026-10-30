#include "voxel_zone.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace {
using clk = std::chrono::steady_clock;
double msSince(clk::time_point t) { return std::chrono::duration<double, std::milli>(clk::now() - t).count(); }
constexpr int BLOCKS_PER_COLUMN = world::CHUNK / world::VOXELS_PER_BLOCK;   // столбец куска - 2×2 блока
constexpr int UNLOAD_MARGIN = 4;    // готовые столбцы за окном выгружаются только дальше: качание у края не строит заново
constexpr int RESORT_FRAMES = 15;   // камера повернулась - очередь пересортировать (видимое - раньше)
// Р26: подземные куски (только с пещерами) - в столбцах ±UNDER_R вокруг камеры и на UNDER_DEPTH кусков вниз от неё;
// глубина округляется до UNDER_STEP кусков - столбцы перестраиваются раз в 16 блоков спуска, а не на каждом шаге.
// Р27: и по горизонтали центр яруса округляется до UNDER_STEP столбцов - край яруса (там заглушки) сдвигается
// раз в 16 блоков; камера от края - не ближе UNDER_R - UNDER_STEP столбцов (16 блоков)
constexpr int UNDER_R = 16, UNDER_DEPTH = 16, UNDER_STEP = 8;
int32_t quantize(int32_t v) { return int32_t(std::floor(float(v) / UNDER_STEP)) * UNDER_STEP + UNDER_STEP / 2; }
}  // namespace

VoxelZone::VoxelZone(render::Backend& gpu, Builder& builder, int radiusColumns)
    : gpu_(gpu), builder_(builder), radius_(std::max(1, radiusColumns)), side_(2 * std::max(1, radiusColumns)) {
    mask_.assign(size_t(side_) * side_, 0);
    sat_.assign(size_t(side_ + 1) * (side_ + 1), 0);
}

VoxelZone::~VoxelZone() {
    builder_.setColumnJobs({});
    for (auto& [k, c] : ready_) release(c);
}

bool VoxelZone::inWindow(int32_t cx, int32_t cz, int margin) const {
    return cx >= ccx_ - radius_ - margin && cx < ccx_ + radius_ + margin && cz >= ccz_ - radius_ - margin &&
           cz < ccz_ + radius_ + margin;
}

void VoxelZone::release(Column& c) {
    for (const ChunkMesh& m : c.chunks) {
        gpu_.destroyMesh(m.mesh);
        --stats_.chunks;
        stats_.quads -= m.quads;
    }
    c.chunks.clear();
}

bool VoxelZone::inUnder(int32_t cx, int32_t cz) const {
    return cx >= qx_ - UNDER_R && cx < qx_ + UNDER_R && cz >= qz_ - UNDER_R && cz < qz_ + UNDER_R;
}

int32_t VoxelZone::wantMin(int32_t cx, int32_t cz) const { return inUnder(cx, cz) ? depthMin_ : Builder::NO_DEPTH; }

uint8_t VoxelZone::wantCaps(int32_t cx, int32_t cz) const {
    if (!inUnder(cx, cz)) return 0;
    return uint8_t((cx == qx_ - UNDER_R ? Builder::CAP_NX : 0) | (cx == qx_ + UNDER_R - 1 ? Builder::CAP_PX : 0) |
                   (cz == qz_ - UNDER_R ? Builder::CAP_NZ : 0) | (cz == qz_ + UNDER_R - 1 ? Builder::CAP_PZ : 0));
}

void VoxelZone::moveWindow(int32_t ccx, int32_t ccz) {
    const int32_t oldX = ccx_ - radius_, oldZ = ccz_ - radius_;
    const bool first = ccx_ == INT32_MIN;
    ccx_ = ccx;
    ccz_ = ccz;
    // готовые столбцы за запасом - выгрузить
    for (auto it = ready_.begin(); it != ready_.end();) {
        if (inWindow(int32_t(it->first >> 32), int32_t(uint32_t(it->first)), UNLOAD_MARGIN)) { ++it; continue; }
        release(it->second);
        it = ready_.erase(it);
    }
    // маска - сдвигом: что было в старом окне, переносим; новые клетки - по таблице готовых
    std::vector<uint8_t> next(size_t(side_) * side_, 0);
    const int32_t newX = ccx - radius_, newZ = ccz - radius_;
    for (int z = 0; z < side_; ++z)
        for (int x = 0; x < side_; ++x) {
            const int32_t cx = newX + x, cz = newZ + z;
            const int ox = cx - oldX, oz = cz - oldZ;
            if (!first && ox >= 0 && oz >= 0 && ox < side_ && oz < side_) next[size_t(z) * side_ + x] = mask_[size_t(oz) * side_ + ox];
            else next[size_t(z) * side_ + x] = ready_.count(key(cx, cz)) ? 1 : 0;
        }
    mask_.swap(next);
    maskDirty_ = true;
}

void VoxelZone::reprioritize(glm::vec3 eye, const Frustum& fr) {
    // ждущие столбцы окна - по весу: квадрат расстояния, вне взгляда - вчетверо дальше; ближние - в конец.
    // Готовые столбцы рядом с камерой, построенные мельче нужного (Р26: подземный ярус), - тоже в очередь.
    // Р27: и столбцы яруса, у которых сменились заглушки (был краем - стал серединой или наоборот).
    struct Job { int64_t w; int32_t cx, cz, cyMin; uint8_t caps; };
    std::vector<Job> jobs;
    for (int z = 0; z < side_; ++z)
        for (int x = 0; x < side_; ++x) {
            const int32_t cx = ccx_ - radius_ + x, cz = ccz_ - radius_ + z;
            const int32_t want = wantMin(cx, cz);
            const uint8_t caps = wantCaps(cx, cz);
            if (mask_[size_t(z) * side_ + x]) {
                auto it = ready_.find(key(cx, cz));
                if (it == ready_.end()) continue;
                const bool shallow = it->second.builtMin > want;
                const bool recap = want != Builder::NO_DEPTH && it->second.caps != caps;
                // Р37: ушёл из подземного яруса - перестроить без пещер (вход в пустоту просвечивал бы)
                const bool closed = want == Builder::NO_DEPTH && it->second.builtMin != Builder::NO_DEPTH;
                if (!shallow && !recap && !closed) continue;
            }
            const int64_t dx = cx - ccx_, dz = cz - ccz_;
            const float bx = float(cx * BLOCKS_PER_COLUMN), bz = float(cz * BLOCKS_PER_COLUMN);
            // высота столбца до постройки неизвестна - коробка по правдоподобным высотам рельефа
            const bool seen = fr.visible({bx, eye.y - 300.0f, bz}, {bx + BLOCKS_PER_COLUMN, eye.y + 300.0f, bz + BLOCKS_PER_COLUMN});
            jobs.push_back({(dx * dx + dz * dz) * (seen ? 1 : 4), cx, cz, want, caps});
        }
    std::sort(jobs.begin(), jobs.end(), [](const Job& a, const Job& b) { return a.w > b.w; });
    std::vector<Builder::ColumnJob> order;
    order.reserve(jobs.size());
    for (const Job& j : jobs) order.push_back({j.cx, j.cz, j.cyMin, j.caps});
    builder_.setColumnJobs(std::move(order));
    sinceSort_ = 0;
}

void VoxelZone::update(glm::vec3 eye, const Frustum& fr, double uploadBudgetMs) {
    const int32_t ccx = int32_t(std::floor(eye.x / BLOCKS_PER_COLUMN)), ccz = int32_t(std::floor(eye.z / BLOCKS_PER_COLUMN));
    const auto worst = [&](int i, clk::time_point t) { stats_.worstMs[i] = std::max(stats_.worstMs[i], msSince(t)); };
    // Р26: глубина подземного яруса - от куска камеры вниз, округлённо
    const int32_t ccy = int32_t(std::floor(eye.y / BLOCKS_PER_COLUMN));
    const int32_t depthMin = int32_t(std::floor(float(ccy - UNDER_DEPTH) / UNDER_STEP)) * UNDER_STEP;
    const bool deeper = depthMin != depthMin_;
    depthMin_ = depthMin;
    // Р27: центр подземного яруса - округлённо; сдвинулся - у столбцов меняются заглушки
    const int32_t qx = quantize(ccx), qz = quantize(ccz);
    const bool shifted = qx != qx_ || qz != qz_;
    qx_ = qx;
    qz_ = qz;
    if (ccx != ccx_ || ccz != ccz_ || deeper || shifted) {
        auto t = clk::now();
        moveWindow(ccx, ccz);
        worst(0, t);
        t = clk::now();
        reprioritize(eye, fr);
        worst(1, t);
    } else if (++sinceSort_ >= RESORT_FRAMES && stats_.pending > 0) {
        const auto t = clk::now();
        reprioritize(eye, fr);
        worst(1, t);
    }

    // готовое - в видеопамять, пока не вышел бюджет кадра
    const auto t0 = clk::now();
    Builder::ColumnResult r;
    while (msSince(t0) < uploadBudgetMs && builder_.popColumn(r)) {
        stats_.workMs += r.ms;
        ++stats_.built;
        // успел уйти за запас - не нужен; уже есть такой же или глубже с нужными заглушками - тоже не нужен;
        // иначе (глубже, или заглушки теперь другие) - заменяет прежний
        if (!inWindow(r.cx, r.cz, UNLOAD_MARGIN)) continue;
        auto old = ready_.find(key(r.cx, r.cz));
        if (old != ready_.end()) {
            const bool deeperNow = r.cyMin < old->second.builtMin;
            const bool capsFixed = r.caps == wantCaps(r.cx, r.cz) && old->second.caps != r.caps;
            // Р37: столбец, ушедший из яруса, - перестроенный без пещер заменяет прежний
            const bool closedNow = r.cyMin == Builder::NO_DEPTH && wantMin(r.cx, r.cz) == Builder::NO_DEPTH &&
                                   old->second.builtMin != Builder::NO_DEPTH;
            if (!deeperNow && !capsFixed && !closedNow) continue;
            release(old->second);
        }
        Column& c = ready_[key(r.cx, r.cz)];
        c.builtMin = r.cyMin;
        c.caps = r.caps;
        for (const Builder::ChunkData& d : r.chunks) {
            c.chunks.push_back({d.origin, gpu_.uploadChunkMesh(d.verts, d.waterQuads), d.quads, d.underground});
            ++stats_.chunks;
            stats_.quads += d.quads;
        }
        if (inWindow(r.cx, r.cz)) {
            mask_[size_t(r.cz - (ccz_ - radius_)) * side_ + size_t(r.cx - (ccx_ - radius_))] = 1;
            maskDirty_ = true;
        }
    }
    worst(2, t0);
    if (maskDirty_) {
        const auto t = clk::now();
        finishMask();
        worst(3, t);
    }
}

void VoxelZone::finishMask() {
    // таблица сумм: S[z+1][x+1] = m + S[z][x+1] + S[z+1][x] - S[z][x]
    const int w = side_ + 1;
    int ready = 0;
    for (int z = 0; z < side_; ++z)
        for (int x = 0; x < side_; ++x) {
            const int m = mask_[size_t(z) * side_ + x];
            ready += m;
            sat_[size_t(z + 1) * w + x + 1] = m + sat_[size_t(z) * w + x + 1] + sat_[size_t(z + 1) * w + x] - sat_[size_t(z) * w + x];
        }
    stats_.ready = ready;
    stats_.pending = side_ * side_ - ready;
    gpu_.updateVoxelMask(side_, mask_);
    maskDirty_ = false;
}

bool VoxelZone::covered(int64_t x0, int64_t z0, int64_t size) const {
    if (ccx_ == INT32_MIN) return false;
    // столбцы, задетые прямоугольником [x0, x0 + size): от floor(x0 / 2) до ceil((x0 + size) / 2)
    const auto floorDiv = [](int64_t b) { return b >= 0 ? b / BLOCKS_PER_COLUMN : -((-b + BLOCKS_PER_COLUMN - 1) / BLOCKS_PER_COLUMN); };
    const int64_t a = floorDiv(x0) - (ccx_ - radius_), b = floorDiv(z0) - (ccz_ - radius_);
    const int64_t c = floorDiv(x0 + size + BLOCKS_PER_COLUMN - 1) - (ccx_ - radius_);
    const int64_t d = floorDiv(z0 + size + BLOCKS_PER_COLUMN - 1) - (ccz_ - radius_);
    if (a < 0 || b < 0 || c > side_ || d > side_ || a >= c || b >= d) return false;
    const int w = side_ + 1;
    const auto S = [&](int64_t z, int64_t x) { return int64_t(sat_[size_t(z) * w + size_t(x)]); };
    return S(d, c) - S(b, c) - S(d, a) + S(b, a) == (c - a) * (d - b);
}

glm::vec4 VoxelZone::window() const {
    return {float((ccx_ - radius_) * BLOCKS_PER_COLUMN), float((ccz_ - radius_) * BLOCKS_PER_COLUMN), float(side_),
            float(BLOCKS_PER_COLUMN)};
}

int VoxelZone::draw(const Frustum& fr, render::CommandList& out) const {
    int n = 0;
    for (const auto& [k, c] : ready_) {
        const int32_t cx = int32_t(k >> 32), cz = int32_t(uint32_t(k));
        if (!inWindow(cx, cz)) continue;   // за окном - там гладкий рельеф
        // Р27: подземные куски - только в ярусе вокруг камеры (вне его их заглушки уже не те - просвечивали бы)
        const bool under = inUnder(cx, cz);
        for (const ChunkMesh& m : c.chunks) {
            if (m.underground && !under) continue;
            const glm::vec3 lo = glm::vec3(m.origin) / float(world::VOXELS_PER_BLOCK);
            if (!fr.visible(lo, lo + glm::vec3(float(BLOCKS_PER_COLUMN)))) continue;
            out.drawChunk(m.mesh, m.origin);
            ++n;
        }
    }
    return n;
}
