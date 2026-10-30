#version 460
// Р65: спрайт дуба вдали - прямоугольник стоймя, повёрнут к камере вокруг вертикали. Экземпляр: a0 = основание
// (x, y, z - в прежних мерах, 16 блоков), высота; a1 = ширина, семя, 0, 0.
layout(location = 0) in vec4 aBase;
layout(location = 1) in vec4 aInfo;

layout(std140, binding = 0) uniform Frame {
    mat4 viewProj;
    vec4 sunDir;
    vec4 eye;
    vec4 voxelZone;
    vec4 fade;
    vec4 sky;
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
    gl_Position = viewProj * vec4(p, 1.0);
    vUv = c;
    vWorld = p;
    vSeed = aInfo.y;
}
