#version 460
// Р109: ТРАВЫ - КОПИИ ГОТОВЫХ МОДЕЛЕЙ. Сетка одной модели растения (мелкие воксели, корень в (16, 0, 16))
// рисуется сразу во многих местах: на копию - место, поворот, размер и сила ветра. Верхушки качаются ветром.
// Фрагменты - terrain.frag (материалы трав - палитра).
#ifdef VULKAN
#define gl_VertexID gl_VertexIndex
#endif
layout(location = 0) in uint aPacked;
layout(location = 1) in vec4 aI0;   // x, y, z (прежние меры от начала видеокарты), поворот вокруг вертикали
layout(location = 2) in vec4 aI1;   // размер, ветер, -, -

layout(std140, binding = 0) uniform Frame {
    mat4 viewProj;
    vec4 sunDir;
    vec4 eye;
    vec4 voxelZone;
    vec4 fade;
    vec4 sky;
    vec4 flowArea;
    vec4 timeV;
    vec4 simArea;
    mat4 invViewProj;
    ivec4 originV;
    vec4 originOld;
    vec4 skyArea;
};

layout(location = 0) out vec4 vColor;
layout(location = 1) out vec3 vWorld;
layout(location = 2) flat out uint vNormal;
layout(location = 3) flat out uint vWater;
layout(location = 4) out float vFoam;
layout(location = 5) flat out uint vMat;
layout(location = 6) out float vLight;
layout(location = 7) out vec3 vBlock;

const vec3 NORMALS[6] = vec3[](vec3(1, 0, 0), vec3(-1, 0, 0), vec3(0, 1, 0), vec3(0, -1, 0), vec3(0, 0, 1), vec3(0, 0, -1));
// палитра трав (материалы 164..185 - как в terrain.vert)
const vec3 PAL[22] = vec3[](
    vec3(0.42, 0.63, 0.22), vec3(0.53, 0.70, 0.28), vec3(0.10, 0.26, 0.15), vec3(0.14, 0.33, 0.18), vec3(0.45, 0.56, 0.38),
    vec3(0.52, 0.62, 0.44), vec3(0.36, 0.60, 0.20), vec3(0.44, 0.66, 0.24), vec3(0.78, 0.12, 0.10),
    vec3(0.33, 0.55, 0.20), vec3(0.42, 0.62, 0.24), vec3(0.95, 0.82, 0.15), vec3(0.35, 0.42, 0.85), vec3(0.85, 0.30, 0.45),
    vec3(0.95, 0.95, 0.92), vec3(0.30, 0.50, 0.22), vec3(0.20, 0.38, 0.14), vec3(0.28, 0.52, 0.18), vec3(0.30, 0.42, 0.16),
    vec3(0.55, 0.58, 0.32), vec3(0.38, 0.24, 0.14), vec3(0.82, 0.72, 0.40));

void main() {
    uint p = aPacked;
    vec3 local = vec3(p & 63u, (p >> 6) & 63u, (p >> 12) & 63u);
    uint normal = (p >> 18) & 7u;
    uint material = (p >> 21) & 2047u;
    float s = aI1.x;
    // мелкие воксели от корня -> прежние меры (1/256), с размером копии
    vec3 q = (local - vec3(16.0, 0.0, 16.0)) * (s / 256.0);
    // ветер: верхушка качается, у корня неподвижно
    float hgt = local.y / 16.0;
    float w = aI1.y * hgt * hgt * (sin(timeV.x * 1.7 + aI0.x * 9.0 + aI0.z * 7.0) * 0.6 + sin(timeV.x * 3.1 + aI0.x * 21.0) * 0.25);
    float c = cos(aI0.w), si = sin(aI0.w);
    vec3 r = vec3(c * q.x - si * q.z, q.y, si * q.x + c * q.z);
    r.x += w * 0.004;
    r.z += w * 0.0025;
    vec3 rel = aI0.xyz + r;
    gl_Position = viewProj * vec4(rel, 1.0);
    vec3 n0 = NORMALS[normal];
    vec3 n = vec3(c * n0.x - si * n0.z, n0.y, si * n0.x + c * n0.z);
    // трава просвечивает: свет мягче, чем у камня, и снизу не чернеет
    float light = 0.5 + 0.5 * max(dot(n, normalize(sunDir.xyz)), 0.0);
    uint mi = clamp(material, 164u, 185u) - 164u;
    vColor = vec4(PAL[mi] * light, 1.0);
    vMat = material;
    vLight = light;
    vWorld = rel + originOld.xyz;
    vNormal = normal;
    vWater = 0u;
    vFoam = 0.0;
    vBlock = rel * 16.0;
}
