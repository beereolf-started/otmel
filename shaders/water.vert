#version 460
// Море (Р25): одна плоскость на уровне моря, едет за камерой; глубина сама прячет её под сушей.
layout(std140, binding = 0) uniform Frame {
    mat4 viewProj;
    vec4 sunDir;
    vec4 eye;
    vec4 voxelZone;
    vec4 fade;
    vec4 sky;
};
layout(location = 0) uniform vec2 uWater;   // уровень (блоки), полуразмер плоскости (блоки)

layout(location = 0) out vec3 vWorld;

void main() {
    // два треугольника из номера вершины: 0..5 -> углы квадрата
    const vec2 CORNERS[6] = vec2[](vec2(-1, -1), vec2(-1, 1), vec2(1, 1), vec2(-1, -1), vec2(1, 1), vec2(1, -1));
    vec2 xz = eye.xz + CORNERS[gl_VertexID] * uWater.y;
    vWorld = vec3(xz.x, uWater.x, xz.y);
    gl_Position = viewProj * vec4(vWorld, 1.0);
}
