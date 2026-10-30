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
layout(location = 0) uniform ivec3 uChunk;   // начало куска в вокселях; у мелкого куска - блок-основа
layout(location = 1) uniform ivec3 uFineOff; // Р56: мелкий кусок - сдвиг от основы в мелких вокселях (1/16 блока)
layout(location = 2) uniform int uFine;      // Р56: 1 - мелкий кусок (дерево и прочее мельче блока)
layout(binding = 1) uniform sampler2D uFlow;   // Р52: скорость воды, блоки/с (x, z) - волны на быстрой воде

layout(location = 0) out vec4 vColor;
layout(location = 1) out vec3 vWorld;          // Р43: точка в мире, блоки
layout(location = 2) flat out uint vNormal;    // Р43: 0 +X, 1 -X, 2 +Y, 3 -Y, 4 +Z, 5 -Z
layout(location = 3) flat out uint vWater;     // Р43: 1 - вода

// цвета материалов, пока без текстур: 0 воздух, 1 камень, 2 земля, 3 трава, 4 песок, 5 снег, 6 вода (Р28)
// Р53: 7 кора, 8 древесина, 9 листва, 10 листва светлее
const vec3 COLORS[11] = vec3[](vec3(1, 0, 1), vec3(0.47, 0.47, 0.5), vec3(0.45, 0.32, 0.2), vec3(0.36, 0.58, 0.25), vec3(0.85, 0.8, 0.55), vec3(0.93, 0.95, 0.98), vec3(0.3, 0.62, 0.92),
                              vec3(0.31, 0.24, 0.17), vec3(0.68, 0.53, 0.33), vec3(0.22, 0.42, 0.14), vec3(0.32, 0.53, 0.2));
// Р34: вода полупрозрачная голубая - сквозь неё видно дно; та же доля - у дальних плиток (tile.vert)
const float WATER_ALPHA = 0.55;
const vec3 NORMALS[6] = vec3[](vec3(1, 0, 0), vec3(-1, 0, 0), vec3(0, 1, 0), vec3(0, -1, 0), vec3(0, 0, 1), vec3(0, 0, -1));

float hash(vec2 q) { return fract(sin(dot(q, vec2(127.1, 311.7))) * 43758.5453); }
float valueNoise(vec2 q) {
    vec2 i = floor(q), f = fract(q);
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash(i), hash(i + vec2(1, 0)), f.x), mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), f.x), f.y);
}

void main() {
    uint p = aPacked;
    vec3 local = vec3(p & 63u, (p >> 6) & 63u, (p >> 12) & 63u);
    uint normal = (p >> 18) & 7u;
    uint material = p >> 21;
    // мир - в прежних мерах (16 блоков мира, Р54); мелкий кусок - ещё в 16 раз мельче (воксель = тексель)
    vec3 world = uFine == 1 ? (vec3(uChunk) + (local + vec3(uFineOff)) / 16.0) / 16.0 : (local + vec3(uChunk)) / 16.0;
    if (material == 6u) {
        // Р52: волны на быстрой воде (скаты, пороги, водопады): вершины воды опускаются бегущим по течению шумом,
        // время - кадрами по 1/8 с. Р55: шаг - тексель (1/16 блока, Р54: воксель = блок), размах - от скорости
        // (с 1 до 6 м/с - до блока), рисунок волн - в блоках мира. Только вниз: у берега вода не встаёт выше
        // края, низ стенок уходит в землю. Соседние грани делят вершины - щелей нет.
        vec2 uv = (world.xz - flowArea.xy) / max(flowArea.z, 1.0);
        vec2 v = all(greaterThanEqual(uv, vec2(0))) && all(lessThanEqual(uv, vec2(1))) ? textureLod(uFlow, uv, 0.0).xy : vec2(0);
        float sp = length(v);
        float amp = clamp((sp - 1.0) / 5.0, 0.0, 1.0) * 16.0;   // в текселях
        if (amp > 0.0) {
            float t = floor(timeV.x * 8.0) / 8.0;
            vec2 q = world.xz * 16.0 * 0.45 - v * t * 0.45;
            float n = valueNoise(q) * 0.65 + valueNoise(q * 2.7 + 3.1) * 0.35;
            world.y -= floor(amp * n + 0.5) / 256.0;
        }
    }
    gl_Position = viewProj * vec4(world, 1.0);
    // свет: рассеянный по нормали плюс постоянная подсветка неба
    float light = 0.35 + 0.65 * max(dot(NORMALS[normal], normalize(sunDir.xyz)), 0.0);
    vColor = vec4(COLORS[min(material, 10u)] * light, material == 6u ? WATER_ALPHA : 1.0);
    vWorld = world;
    vNormal = normal;
    vWater = material == 6u ? 1u : 0u;
}
