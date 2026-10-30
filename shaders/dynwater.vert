#version 460
// Р45: вода автомата у камеры - своя сетка: вершина - точка в мире (блоки) и номер нормали. Фрагменты - тот же
// шейдер, что у воды кусков (terrain.frag): рябь по течению, пена, падающая вода на стенках.
layout(location = 0) in vec4 aPos;   // xyz - блоки, w - нормаль 0..5

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
};

layout(location = 0) out vec4 vColor;
layout(location = 1) out vec3 vWorld;
layout(location = 2) flat out uint vNormal;
layout(location = 3) flat out uint vWater;
layout(location = 4) out float vFoam;

const vec3 WATER = vec3(0.3, 0.62, 0.92);
const vec3 NORMALS[6] = vec3[](vec3(1, 0, 0), vec3(-1, 0, 0), vec3(0, 1, 0), vec3(0, -1, 0), vec3(0, 0, 1), vec3(0, 0, -1));

void main() {
    gl_Position = viewProj * vec4(aPos.xyz, 1.0);
    uint n = uint(aPos.w + 0.5);
    // Р52: нормаль 6..11 - воксель брызг (капли над быстрой водой): белая пена
    bool spray = n >= 6u;
    if (spray) n -= 6u;
    float light = 0.35 + 0.65 * max(dot(NORMALS[n], normalize(sunDir.xyz)), 0.0);
    vColor = spray ? vec4(vec3(0.86, 0.92, 0.97) * (0.6 + 0.4 * light), 0.85) : vec4(WATER * light, 0.55);
    vWorld = aPos.xyz;
    vNormal = n;
    vFoam = 0.0;
    vWater = spray ? 3u : 2u;   // 2 - вода автомата (Р45), 3 - брызги
}
