#version 460
#ifdef VULKAN   // Р73
#define gl_VertexID gl_VertexIndex
#endif
// Р65: спрайт дуба вдали - прямоугольник стоймя, повёрнут к камере вокруг вертикали. Экземпляр: a0 = основание
// (x, y, z - в прежних мерах, 16 блоков), высота; a1 = ширина, семя, 0, 0.
layout(location = 0) in vec4 aBase;
layout(location = 1) in vec4 aInfo;

layout(std140, binding = 0) uniform Frame {
    mat4 viewProj;     // камера - относительно originOld (Р69)
    vec4 sunDir;
    vec4 eye;          // камера, прежние меры (16 блоков), абсолютно
    vec4 voxelZone;
    vec4 fade;
    vec4 sky;
    vec4 flowArea;     // Р43: поле течений - угол x, z, сторона (блоки)
    vec4 timeV;        // Р43: x - секунды
    vec4 simArea;      // Р58: зона мелкой воды
    mat4 invViewProj;  // Р51: из экрана - в координаты относительно originOld
    ivec4 originV;     // Р69: начало координат видеокарты - воксели (блоки мира), кратно куску
    vec4 originOld;    // Р69: оно же в прежних мерах (originV / 16)
};

layout(location = 0) out vec2 vUv;
layout(location = 1) out vec3 vWorld;
layout(location = 2) flat out float vSeed;

void main() {
    const vec2 C[6] = vec2[](vec2(0, 0), vec2(1, 0), vec2(1, 1), vec2(0, 0), vec2(1, 1), vec2(0, 1));
    vec2 c = C[gl_VertexID];
    vec3 base = aBase.xyz;
    vec2 to = eye.xz - base.xz;
    vec2 side = normalize(vec2(-to.y, to.x) + vec2(1e-6, 0));
    vec3 p = base + vec3(side.x, 0, side.y) * (c.x - 0.5) * aInfo.x + vec3(0, c.y * aBase.w, 0);
    gl_Position = viewProj * vec4(p - originOld.xyz, 1.0);   // Р69
    vUv = c;
    vWorld = p;
    vSeed = aInfo.y;
}
