#version 460
// Р59: брызги и водяная пыль - кубики, экземпляр - капля: a0 = (x, y, z) в прежних мерах (16 блоков), размер;
// a1 = (непрозрачность, вид: 0 капля, 1 пыль). Три грани к камере (18 вершин), фрагменты - terrain.frag (брызги).
layout(location = 0) in vec4 aPos;
layout(location = 1) in vec4 aInfo;

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

const vec3 NORMALS[6] = vec3[](vec3(1, 0, 0), vec3(-1, 0, 0), vec3(0, 1, 0), vec3(0, -1, 0), vec3(0, 0, 1), vec3(0, 0, -1));

void main() {
    int face = gl_VertexID / 6, k = int[](0, 1, 2, 0, 2, 3)[gl_VertexID % 6];
    float s = aPos.w;
    vec3 b = floor(aPos.xyz * 256.0) / 256.0;   // по сетке текселей
    int ax = face;                               // 0 x, 1 y, 2 z
    bool plus = eye[ax] > b[ax] + s * 0.5;
    int u = (ax + 1) % 3, w = (ax + 2) % 3;
    vec3 p = b;
    p[ax] += plus ? s : 0.0;
    p[u] += (k == 1 || k == 2) ? s : 0.0;
    p[w] += k >= 2 ? s : 0.0;
    gl_Position = viewProj * vec4(p, 1.0);
    uint n = uint(ax * 2 + (plus ? 0 : 1));
    float light = 0.6 + 0.4 * max(dot(NORMALS[n], normalize(sunDir.xyz)), 0.0);
    vec3 c = aInfo.y > 0.5 ? vec3(0.9, 0.94, 0.97) : vec3(0.86, 0.92, 0.97);
    vColor = vec4(c * light, aInfo.x);
    vWorld = p;
    vNormal = n;
    vWater = 3u;
    vFoam = 0.0;
}
