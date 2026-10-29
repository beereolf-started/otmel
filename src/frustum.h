#pragma once
// Пирамида видимости из матрицы камеры (Гриббс - Хартманн): шесть плоскостей, точка внутри - если со всеми >= 0.
// Глубина у нас в [0, 1] (обратная, Р20), поэтому плоскости по z - строка 2 и строка 3 минус строка 2.
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

struct Frustum {
    glm::vec4 planes[6];
    explicit Frustum(const glm::mat4& m) {
        const auto row = [&](int r) { return glm::vec4(m[0][r], m[1][r], m[2][r], m[3][r]); };
        planes[0] = row(3) + row(0);
        planes[1] = row(3) - row(0);
        planes[2] = row(3) + row(1);
        planes[3] = row(3) - row(1);
        planes[4] = row(2);
        planes[5] = row(3) - row(2);
    }
    // коробка видна, если её самый «передний» угол (по нормали плоскости) не позади ни одной плоскости
    bool visible(glm::vec3 lo, glm::vec3 hi) const {
        for (const glm::vec4& p : planes) {
            const glm::vec3 v(p.x >= 0 ? hi.x : lo.x, p.y >= 0 ? hi.y : lo.y, p.z >= 0 ? hi.z : lo.z);
            if (p.x * v.x + p.y * v.y + p.z * v.z + p.w < 0) return false;
        }
        return true;
    }
};
