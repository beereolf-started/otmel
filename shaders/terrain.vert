#version 460
// Вершина рельефа - одно 32-битное число (Р17): позиция в куске 3×6 бит, нормаль 3 бита, материал 11 бит.
layout(location = 0) in uint aPacked;

layout(std140, binding = 0) uniform Frame {
    mat4 viewProj;     // камера
    vec4 sunDir;       // направление на солнце (xyz)
    vec4 eye;          // остальное - для плиток дальних уровней (tile.vert)
    vec4 voxelZone;
    vec4 fade;
    vec4 sky;
    vec4 flowArea;     // Р43: поле течений - угол x, z, сторона (блоки)
    vec4 timeV;        // Р43: x - секунды
    vec4 simArea;      // Р45: зона автомата воды - угол x, z, сторона (блоки), w - 1, если есть
};
layout(location = 0) uniform ivec3 uChunk;   // начало куска в вокселях

layout(location = 0) out vec4 vColor;
layout(location = 1) out vec3 vWorld;          // Р43: точка в мире, блоки
layout(location = 2) flat out uint vNormal;    // Р43: 0 +X, 1 -X, 2 +Y, 3 -Y, 4 +Z, 5 -Z
layout(location = 3) flat out uint vWater;     // Р43: 1 - вода

// цвета материалов, пока без текстур: 0 воздух, 1 камень, 2 земля, 3 трава, 4 песок, 5 снег, 6 вода (Р28)
const vec3 COLORS[7] = vec3[](vec3(1, 0, 1), vec3(0.47, 0.47, 0.5), vec3(0.45, 0.32, 0.2), vec3(0.36, 0.58, 0.25), vec3(0.85, 0.8, 0.55), vec3(0.93, 0.95, 0.98), vec3(0.3, 0.62, 0.92));
// Р34: вода полупрозрачная голубая - сквозь неё видно дно; та же доля - у дальних плиток (tile.vert)
const float WATER_ALPHA = 0.55;
const vec3 NORMALS[6] = vec3[](vec3(1, 0, 0), vec3(-1, 0, 0), vec3(0, 1, 0), vec3(0, -1, 0), vec3(0, 0, 1), vec3(0, 0, -1));

void main() {
    uint p = aPacked;
    vec3 local = vec3(p & 63u, (p >> 6) & 63u, (p >> 12) & 63u);
    uint normal = (p >> 18) & 7u;
    uint material = p >> 21;
    vec3 world = (local + vec3(uChunk)) / 16.0;          // воксель = 1/16 блока (Р13): мир - в блоках
    gl_Position = viewProj * vec4(world, 1.0);
    // свет: рассеянный по нормали плюс постоянная подсветка неба
    float light = 0.35 + 0.65 * max(dot(NORMALS[normal], normalize(sunDir.xyz)), 0.0);
    vColor = vec4(COLORS[min(material, 6u)] * light, material == 6u ? WATER_ALPHA : 1.0);
    vWorld = world;
    vNormal = normal;
    vWater = material == 6u ? 1u : 0u;
}
