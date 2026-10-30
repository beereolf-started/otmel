#pragma once
// Сетка куска (Р17): видимые грани - битовыми масками по столбцам, затем слияние одинаковых граней
// в прямоугольники (greedy meshing). Вершина - одно 32-битное число:
//   биты  0..5  x в куске (0..32)      биты 18..20 нормаль (0..5: +X -X +Y -Y +Z -Z)
//   биты  6..11 y                      биты 21..31 материал
//   биты 12..17 z
#include <cstdint>
#include <vector>

#include "world/chunk.h"
#include "world/terrain.h"

namespace world {

// quads - всего; последние waterQuads прямоугольников сетки - вода (Р34: прозрачна, рисуется отдельным проходом)
struct MeshStats { int quads = 0, waterQuads = 0; };

// Собрать сетку куска. Соседей за краем куска узнаём по карте высот с каймой (heights): тот же рельеф,
// что и у соседа, без доступа к его вокселям. Вершины - по 4 на прямоугольник.
// Правленый кусок (c.edited) - маски и материалы из его вокселей; нетронутый - всё формулой от высот.
void buildMesh(const Chunk& c, const ColumnHeights& heights, std::vector<uint32_t>& vertices, MeshStats& stats);
// Нетронутый кусок по одному положению: массив вокселей не нужен вовсе (Р2, Р22).
// air - воздух пещер в куске с каймой (Р26, world/caves.h) или nullptr: твёрдо = ниже поверхности И не пещера.
void buildMesh(ChunkPos pos, const ColumnHeights& heights, std::vector<uint32_t>& vertices, MeshStats& stats,
               const uint64_t (*air)[CHUNK + 2] = nullptr);

}  // namespace world
