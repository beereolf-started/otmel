#version 460
// Вершина рельефа - одно 32-битное число (Р17): позиция в куске 3×6 бит, нормаль 3 бита, материал 11 бит.
layout(location = 0) in uint aPacked;

layout(std140, binding = 0) uniform Frame {
    mat4 viewProj;     // камера
    vec4 sunDir;       // направление на солнце (xyz)
};
layout(location = 0) uniform ivec3 uChunk;   // начало куска в вокселях

layout(location = 0) out vec3 vColor;

// цвета материалов, пока без текстур: 0 воздух, 1 камень, 2 земля, 3 трава, 4 песок
const vec3 COLORS[5] = vec3[](vec3(1, 0, 1), vec3(0.47, 0.47, 0.5), vec3(0.45, 0.32, 0.2), vec3(0.36, 0.58, 0.25), vec3(0.85, 0.8, 0.55));
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
    vColor = COLORS[min(material, 4u)] * light;
}
