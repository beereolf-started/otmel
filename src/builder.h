#pragma once
// Общий пул построек мира (Р21): постоянные потоки, две полосы заданий.
//   1) плитки дальнего рельефа - первыми: дешёвые, а без них в мире дыра; грубые уровни раньше мелких;
//   2) столбцы вокселей - по очереди, которую задаёт зона вокселей (ближние и видимые - раньше).
// Кадр сам ничего не строит: только забирает готовое и выгружает в видеопамять.
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include <glm/vec3.hpp>

#include "world/lod.h"
#include "world/terrain.h"

class Builder {
public:
    struct ChunkData { glm::ivec3 origin; std::vector<uint32_t> verts; int quads; };
    // Задание столбца: cyMin - до какого куска строить вниз (подземные куски - только с пещерами, Р26);
    // NO_DEPTH - только куски, через которые проходит поверхность
    static constexpr int32_t NO_DEPTH = INT32_MAX;
    struct ColumnJob { int32_t cx, cz, cyMin; };
    struct ColumnResult { int32_t cx, cz, cyMin; std::vector<ChunkData> chunks; double ms; };
    struct TileResult { world::TileKey key; world::TileMesh mesh; double ms; };

    explicit Builder(const world::TerrainParams& tp);
    ~Builder();

    // очередь столбцов целиком заменяется (ближние - в конце); те, что уже строятся, повторно не встают
    void setColumnJobs(std::vector<ColumnJob> jobs);
    // заказать плитки; уже заказанные и строящиеся не дублируются
    void requestTiles(const std::vector<world::TileKey>& keys);
    bool popColumn(ColumnResult& out);
    bool popTile(TileResult& out);
    int threads() const { return int(threads_.size()); }

private:
    static uint64_t colKey(int32_t cx, int32_t cz) { return uint64_t(uint32_t(cx)) << 32 | uint32_t(cz); }
    void worker();

    const world::TerrainParams& tp_;
    std::mutex mu_;
    std::condition_variable cv_;
    std::vector<ColumnJob> columns_;                        // ближние - в конце
    std::unordered_set<uint64_t> buildingColumns_;
    std::vector<world::TileKey> tiles_;
    std::unordered_set<uint64_t> wantedTiles_;           // заказаны, строятся или готовы, но не забраны
    std::deque<ColumnResult> doneColumns_;
    std::deque<TileResult> doneTiles_;
    bool stop_ = false;
    std::vector<std::thread> threads_;
};
