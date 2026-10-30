#pragma once
// Потревоженная вода (Р43, шаг 3): клеточный автомат на столбцах - модель «виртуальных труб». Вне потревоженных
// зон вода остаётся сгенерированной (без расчёта); в зоне вокруг изменения рельефа - сетка столбцов: у каждого
// дно (земля) и глубина воды, между соседями - потоки. Поток к соседу разгоняется разницей уровней воды
// (g·Δh/ℓ), не бывает отрицательным; если столбец за шаг отдал бы больше, чем в нём есть, все его потоки
// пропорционально урезаются - вода не уходит в минус, объём сохраняется точно (до округления). С уступа вода
// падает сама: разница уровней большая - поток быстрый. Единицы - блоки и секунды.
#include <cstdint>
#include <vector>

namespace world {

class WaterSim {
public:
    // сетка w×h столбцов; ground - высота дна, surface - уровень воды (ниже дна - сухо), блоки
    void init(int w, int h, const std::vector<float>& ground, const std::vector<float>& surface);
    void step(float dt);                         // шаг не больше ~0.05 с
    float volume() const;                        // воды всего, блоки³
    float depth(int x, int z) const { return depth_[idx(x, z)]; }
    float surface(int x, int z) const { return ground_[idx(x, z)] + depth_[idx(x, z)]; }
    void velocity(int x, int z, float& vx, float& vz) const;   // по потокам через столбец, блоки/с
    void setGround(int x, int z, float g) { ground_[idx(x, z)] = g; }   // изменение рельефа (копание, плотина)
    // края сетки держат начальный уровень: река втекает и вытекает (зона вокруг камеры - кусок реки, а не лужа)
    bool holdEdges = false;

private:
    int idx(int x, int z) const { return z * w_ + x; }
    int w_ = 0, h_ = 0;
    std::vector<float> ground_, depth_, flux_;   // flux_: 4 на столбец - к +x, -x, +z, -z
    std::vector<float> depth0_;                  // начальная глубина - для краёв
};

}  // namespace world
