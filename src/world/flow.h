#pragma once
// Поле течений (Р43): вокруг камеры, на блок - скорость воды в блоках в секунду (x, z). Реки - вдоль русла, по
// Маннингу (RiverSet::current: быстрее на крутом и у полноводных); озёра, море, суша - ноль. Его читает шейдер
// воды: рябь сносится по течению, на быстрых местах - пена. Вблизи камеры его уточняет автомат воды (water_sim).
#include <cstdint>
#include <vector>

#include "world/terrain.h"

namespace world {

// Квадрат size×size блоков с углом (bx0, bz0) в блоках; out - 2 числа на блок (vx, vz), строками по z.
// ground, surface (если даны) - верх дна и верх воды столбца в блоках (сухо - воды ниже дна)
void flowField(const TerrainParams& p, int32_t bx0, int32_t bz0, int size, std::vector<float>& out,
               std::vector<float>* ground = nullptr, std::vector<float>* surface = nullptr);

}  // namespace world
