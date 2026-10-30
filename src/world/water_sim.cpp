#include "world/water_sim.h"

#include <algorithm>

namespace world {
namespace {
constexpr float G = 9.8f;       // ускорение, блоки/с² (блок - метр)
constexpr float PIPE = 1.0f;    // сечение трубы между столбцами, блоки²
constexpr float CELL = 1.0f;    // столбец - блок
constexpr float DAMP = 1.0f;    // затухание потоков, 1/с: волны гаснут за секунды, без вечного качания
constexpr int DX[4] = {1, -1, 0, 0}, DZ[4] = {0, 0, 1, -1};
}  // namespace

void WaterSim::init(int w, int h, const std::vector<float>& ground, const std::vector<float>& surface) {
    w_ = w;
    h_ = h;
    ground_ = ground;
    depth_.assign(size_t(w) * h, 0.0f);
    for (size_t k = 0; k < depth_.size(); ++k) depth_[k] = std::max(0.0f, surface[k] - ground[k]);
    flux_.assign(size_t(w) * h * 4, 0.0f);
    depth0_ = depth_;
}

void WaterSim::step(float dt) {
    // 1. потоки: разгон разницей уровней, только наружу; края сетки закрыты
    for (int z = 0; z < h_; ++z)
        for (int x = 0; x < w_; ++x) {
            const int k = idx(x, z);
            const float s = ground_[k] + depth_[k];
            float out = 0;
            for (int d = 0; d < 4; ++d) {
                float& f = flux_[size_t(k) * 4 + d];
                const int nx = x + DX[d], nz = z + DZ[d];
                if (nx < 0 || nz < 0 || nx >= w_ || nz >= h_) { f = 0; continue; }
                const int n = idx(nx, nz);
                f = std::max(0.0f, f * (1.0f - DAMP * dt) + dt * PIPE * G * (s - (ground_[n] + depth_[n])) / CELL);
                out += f;
            }
            // больше, чем есть, за шаг не отдать: все потоки столбца - пропорционально меньше
            const float can = depth_[k] * CELL * CELL;
            if (out * dt > can && out > 0) {
                const float scale = can / (out * dt);
                for (int d = 0; d < 4; ++d) flux_[size_t(k) * 4 + d] *= scale;
            }
        }
    // 2. глубины: приток от соседей минус отток
    for (int z = 0; z < h_; ++z)
        for (int x = 0; x < w_; ++x) {
            const int k = idx(x, z);
            float in = 0, out = 0;
            for (int d = 0; d < 4; ++d) {
                out += flux_[size_t(k) * 4 + d];
                const int nx = x - DX[d], nz = z - DZ[d];   // сосед, чей поток в сторону d идёт сюда
                if (nx >= 0 && nz >= 0 && nx < w_ && nz < h_) in += flux_[size_t(idx(nx, nz)) * 4 + d];
            }
            depth_[k] = std::max(0.0f, depth_[k] + dt * (in - out) / (CELL * CELL));
        }
    if (holdEdges)
        for (int z = 0; z < h_; ++z)
            for (int x = 0; x < w_; ++x)
                if (x == 0 || z == 0 || x == w_ - 1 || z == h_ - 1) depth_[idx(x, z)] = depth0_[idx(x, z)];
}

float WaterSim::volume() const {
    double v = 0;
    for (float d : depth_) v += d;
    return float(v * CELL * CELL);
}

void WaterSim::velocity(int x, int z, float& vx, float& vz) const {
    const int k = idx(x, z);
    const auto fl = [&](int xx, int zz, int d) {
        return xx < 0 || zz < 0 || xx >= w_ || zz >= h_ ? 0.0f : flux_[size_t(idx(xx, zz)) * 4 + d];
    };
    // поток через столбец по оси: (из -x в него + из него в +x) - (из +x в него + из него в -x), пополам
    const float qx = (fl(x - 1, z, 0) + fl(x, z, 0) - fl(x + 1, z, 1) - fl(x, z, 1)) * 0.5f;
    const float qz = (fl(x, z - 1, 2) + fl(x, z, 2) - fl(x, z + 1, 3) - fl(x, z, 3)) * 0.5f;
    const float d = std::max(depth_[k], 0.05f);
    vx = qx / (d * CELL);
    vz = qz / (d * CELL);
}

}  // namespace world
