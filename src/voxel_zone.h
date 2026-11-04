#pragma once
// Зона вокселей за камерой (Р21): квадрат ±R столбцов кусков вокруг столбца камеры. Недостающие столбцы -
// в очередь общего пула (ближние и видимые - раньше), кадр только выгружает готовое в видеопамять с бюджетом
// по времени. Где столбец ещё не готов, остаётся гладкий рельеф той же формы (маска готовых столбцов для
// шейдера плиток) - быстрый полёт не оставляет дыр.
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "builder.h"
#include "world/query.h"
#include "frustum.h"
#include "render/render.h"

class VoxelZone {
public:
    struct Stats {
        int ready = 0, pending = 0;       // столбцов окна: готово, ждут постройки
        int built = 0;                    // столбцов построено с начала
        int64_t chunks = 0, quads = 0;    // в видеопамяти
        double workMs = 0;                // работа потоков с начала (сумма)
        double worstMs[4] = {};           // худшее за кадр: сдвиг окна, очередь, выгрузка, маска
    };
    void resetWorst() { for (double& w : stats_.worstMs) w = 0; }

    VoxelZone(render::Backend& gpu, Builder& builder, int radiusColumns);
    ~VoxelZone();
    // Р73: все сетки - из видеопамяти (до закрытия пути отрисовки)
    void releaseAll();

    // кадр: сдвиг окна за камерой, очередь, выгрузка готового (не дольше uploadBudgetMs), маска для шейдера
    // forward - куда смотрит камера (единичный): за спиной грузим в последнюю очередь
    void update(glm::vec3 eye, glm::vec3 forward, const Frustum& fr, double uploadBudgetMs);
    // куски готовых столбцов окна, видимые камере
    int draw(const Frustum& fr, render::CommandList& out) const;
    // Р83: столбец устарел (в нём сломали блок) - заказать заново. Прежние сетки при этом НЕ
    // снимаются: пока столбец строится, на его месте зияла бы дыра в дальний рельеф.
    void invalidate(int32_t cx, int32_t cz);
    void setEdits(const world::Edits* e) { edits_ = e; }
    // прямоугольник в блоках целиком покрыт готовыми вокселями (таблица сумм по маске, 4 чтения)
    bool covered(int64_t x0, int64_t z0, int64_t size) const;
    // окно для шейдера: minX, minZ (блоки), столбцов по стороне, блоков в столбце
    glm::vec4 window() const;
    const Stats& stats() const { return stats_; }

private:
    struct ChunkMesh { glm::ivec3 origin; render::MeshId mesh; int quads; bool underground; };
    // готовый столбец: его куски, до какой глубины он построен (Builder::NO_DEPTH - только поверхность)
    // и с каких сторон его подземные куски закрыты заглушками (Р27)
    struct Column { std::vector<ChunkMesh> chunks; int32_t builtMin = Builder::NO_DEPTH; uint8_t caps = 0; };

    static uint64_t key(int32_t cx, int32_t cz) { return uint64_t(uint32_t(cx)) << 32 | uint32_t(cz); }
    bool inWindow(int32_t cx, int32_t cz, int margin = 0) const;
    void moveWindow(int32_t ccx, int32_t ccz);
    void reprioritize(glm::vec3 eye, glm::vec3 forward, const Frustum& fr);
    // радиус зоны по горизонтали на нынешней высоте: зона - шар, а не цилиндр
    int32_t horizRadius(glm::vec3 eye) const;
    // Высота земли под камерой (блоки), из её построенного столбца. Пока ни одна колонка не готова,
    // она НЕИЗВЕСТНА - и тогда радиус обязан быть полным. Иначе выходит, что игрок висит на своей
    // абсолютной высоте над нулём, зона схлопывается и мир не грузится вовсе.
    static constexpr float GROUND_UNKNOWN = -1e30f;
    mutable float groundY_ = GROUND_UNKNOWN;
    int32_t rHoriz_ = 1;                   // нынешний радиус зоны по горизонтали (столбцы)
    void release(Column& c);
    void finishMask();

    render::Backend& gpu_;
    Builder& builder_;
    const int radius_, side_;
    int32_t ccx_ = INT32_MIN, ccz_ = INT32_MIN;              // столбец камеры
    std::unordered_map<uint64_t, Column> ready_;   // готовые столбцы (и в запасе за окном)
    // Р83: столбцы, устаревшие из-за правки. Строящийся столбец отменить нельзя - его итог
    // придёт из прежнего мира, и такой итог отбрасывается по числу учтённых правок.
    std::unordered_set<uint64_t> stale_;
    const world::Edits* edits_ = nullptr;
    int32_t depthMin_ = Builder::NO_DEPTH;        // Р26: до какого куска строить вниз рядом с камерой
    int32_t qx_ = INT32_MIN, qz_ = INT32_MIN;     // Р27: центр подземного яруса - камера, округлённо
    bool inUnder(int32_t cx, int32_t cz) const;   // столбец в подземном ярусе
    int32_t wantMin(int32_t cx, int32_t cz) const;
    uint8_t wantCaps(int32_t cx, int32_t cz) const;

    // маска готовых столбцов окна (1 байт на столбец, строка - по z) и таблица сумм по ней
    std::vector<uint8_t> mask_;
    std::vector<int32_t> sat_;
    bool maskDirty_ = true;
    int sinceSort_ = 0;
    Stats stats_;
};
