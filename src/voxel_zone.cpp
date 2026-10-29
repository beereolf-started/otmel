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

void VoxelZone::release(std::vector<ChunkMesh>& c) {
    for (const ChunkMesh& m : c) {
        gpu_.destroyMesh(m.mesh);
        --stats_.chunks;
        stats_.quads -= m.quads;
    }
    c.clear();
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
    // ждущие столбцы окна - по весу: квадрат расстояния, вне взгляда - вчетверо дальше; ближние - в конец
    struct Job { int64_t w; int32_t cx, cz; };
    std::vector<Job> jobs;
    for (int z = 0; z < side_; ++z)
        for (int x = 0; x < side_; ++x) {
            if (mask_[size_t(z) * side_ + x]) continue;
            const int32_t cx = ccx_ - radius_ + x, cz = ccz_ - radius_ + z;
            const int64_t dx = cx - ccx_, dz = cz - ccz_;
            const float bx = float(cx * BLOCKS_PER_COLUMN), bz = float(cz * BLOCKS_PER_COLUMN);
            // высота столбца до постройки неизвестна - коробка по правдоподобным высотам рельефа
            const bool seen = fr.visible({bx, eye.y - 300.0f, bz}, {bx + BLOCKS_PER_COLUMN, eye.y + 300.0f, bz + BLOCKS_PER_COLUMN});
            jobs.push_back({(dx * dx + dz * dz) * (seen ? 1 : 4), cx, cz});
        }
    std::sort(jobs.begin(), jobs.end(), [](const Job& a, const Job& b) { return a.w > b.w; });
    std::vector<std::pair<int32_t, int32_t>> order;
    order.reserve(jobs.size());
    for (const Job& j : jobs) order.push_back({j.cx, j.cz});
    builder_.setColumnJobs(std::move(order));
    sinceSort_ = 0;
}

void VoxelZone::update(glm::vec3 eye, const Frustum& fr, double uploadBudgetMs) {
    const int32_t ccx = int32_t(std::floor(eye.x / BLOCKS_PER_COLUMN)), ccz = int32_t(std::floor(eye.z / BLOCKS_PER_COLUMN));
    const auto worst = [&](int i, clk::time_point t) { stats_.worstMs[i] = std::max(stats_.worstMs[i], msSince(t)); };
    if (ccx != ccx_ || ccz != ccz_) {
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
        // уже есть (вернулся в окно и построен дважды) или успел уйти за запас - не нужен
        if (ready_.count(key(r.cx, r.cz)) || !inWindow(r.cx, r.cz, UNLOAD_MARGIN)) continue;
        std::vector<ChunkMesh>& c = ready_[key(r.cx, r.cz)];
        for (const Builder::ChunkData& d : r.chunks) {
            c.push_back({d.origin, gpu_.uploadChunkMesh(d.verts), d.quads});
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
        if (!inWindow(int32_t(k >> 32), int32_t(uint32_t(k)))) continue;   // за окном - там гладкий рельеф
        for (const ChunkMesh& m : c) {
            const glm::vec3 lo = glm::vec3(m.origin) / float(world::VOXELS_PER_BLOCK);
            if (!fr.visible(lo, lo + glm::vec3(float(BLOCKS_PER_COLUMN)))) continue;
            out.drawChunk(m.mesh, m.origin);
            ++n;
        }
    }
    return n;
}
