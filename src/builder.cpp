#include "builder.h"

#include <algorithm>
#include <chrono>

#include "world/caves.h"
#include "world/mesher.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

namespace {
using clk = std::chrono::steady_clock;
double msSince(clk::time_point t) { return std::chrono::duration<double, std::milli>(clk::now() - t).count(); }
}  // namespace

Builder::Builder(const world::TerrainParams& tp) : tp_(tp) {
    const unsigned n = std::max(1u, std::thread::hardware_concurrency() - 1);   // главный поток рисует
    for (unsigned i = 0; i < n; ++i) threads_.emplace_back(&Builder::worker, this);
}

Builder::~Builder() {
    {
        std::lock_guard lk(mu_);
        stop_ = true;
    }
    cv_.notify_all();
    for (auto& t : threads_) t.join();
}

void Builder::setColumnJobs(std::vector<ColumnJob> jobs) {
    {
        std::lock_guard lk(mu_);
        jobs.erase(std::remove_if(jobs.begin(), jobs.end(),
                                  [&](const ColumnJob& j) { return buildingColumns_.count(colKey(j.cx, j.cz)) > 0; }),
                   jobs.end());
        columns_ = std::move(jobs);
    }
    cv_.notify_all();
}

void Builder::requestTiles(const std::vector<world::TileKey>& keys) {
    bool added = false;
    {
        std::lock_guard lk(mu_);
        for (const world::TileKey& k : keys)
            if (wantedTiles_.insert(world::tileHash(k)).second) { tiles_.push_back(k); added = true; }
    }
    if (added) cv_.notify_all();
}

bool Builder::popColumn(ColumnResult& out) {
    std::lock_guard lk(mu_);
    if (doneColumns_.empty()) return false;
    out = std::move(doneColumns_.front());
    doneColumns_.pop_front();
    return true;
}

bool Builder::popTile(TileResult& out) {
    std::lock_guard lk(mu_);
    if (doneTiles_.empty()) return false;
    out = std::move(doneTiles_.front());
    doneTiles_.pop_front();
    wantedTiles_.erase(world::tileHash(out.key));
    return true;
}

void Builder::worker() {
    // Строители - ниже обычного приоритета: на 4 ядрах (8 потоков) семь строителей на равных отнимали время
    // у потока кадра - рывки до десятков мс в любой его части. Теперь кадр всегда первый, стройка - остаток.
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#endif
    world::ColumnHeights hm;
    std::vector<uint32_t> verts;
    static thread_local uint64_t air[world::CHUNK + 2][world::CHUNK + 2];
    for (;;) {
        bool isTile = false;
        world::TileKey tile{};
        ColumnJob col{};
        {
            std::unique_lock lk(mu_);
            cv_.wait(lk, [&] { return stop_ || !tiles_.empty() || !columns_.empty(); });
            if (stop_) return;
            if (!tiles_.empty()) {
                // плитки - первыми, из них - самая грубая (её четверть закрывает место, пока нет мелких)
                auto it = std::max_element(tiles_.begin(), tiles_.end(),
                                           [](const world::TileKey& a, const world::TileKey& b) { return a.level < b.level; });
                tile = *it;
                *it = tiles_.back();
                tiles_.pop_back();
                isTile = true;
            } else {
                col = columns_.back();
                columns_.pop_back();
                buildingColumns_.insert(colKey(col.cx, col.cz));
            }
        }
        const auto t0 = clk::now();
        if (isTile) {
            TileResult r{tile, {}, 0};
            world::buildTile(tp_, tile, r.mesh);
            r.ms = msSince(t0);
            std::lock_guard lk(mu_);
            doneTiles_.push_back(std::move(r));
            continue;
        }
        ColumnResult r{col.cx, col.cz, col.cyMin, {}, 0};
        world::columnHeights(tp_, col.cx, col.cz, hm);
        // по высоте - куски, через которые проходит поверхность (с каймой), Р2; рядом с камерой - и ниже, до cyMin:
        // там строятся только куски с пещерами (сплошной камень граней не даёт), Р26
        const int cyLo = (hm.minH - 1) >> world::CHUNK_LOG2, cyHi = hm.maxH >> world::CHUNK_LOG2;
        constexpr int32_t WORLD_BOTTOM = -500 * world::VOXELS_PER_BLOCK / world::CHUNK;   // Р10
        const int cyFrom = col.cyMin == NO_DEPTH ? cyLo : std::max(std::min(cyLo, col.cyMin), WORLD_BOTTOM);
        for (int cy = cyFrom; cy <= cyHi; ++cy) {
            const world::ChunkPos pos{col.cx, cy, col.cz};
            const bool caves = world::caveAir(tp_, pos, hm, air);
            if (!caves && cy < cyLo) continue;   // подземный и без пещер - сплошной камень
            // нетронутый кусок: сетка прямо от высот (и пещер), без заполнения вокселей (Р22)
            world::MeshStats ms;
            world::buildMesh(pos, hm, verts, ms, caves ? air : nullptr);
            if (verts.empty()) continue;
            r.chunks.push_back({{col.cx * world::CHUNK, cy * world::CHUNK, col.cz * world::CHUNK}, verts, ms.quads});
        }
        r.ms = msSince(t0);
        std::lock_guard lk(mu_);
        buildingColumns_.erase(colKey(col.cx, col.cz));
        doneColumns_.push_back(std::move(r));
    }
}
