#version 460
// Р59: брызги и водяная пыль - кубики, экземпляр - капля: a0 = (x, y, z) в прежних мерах (16 блоков), размер;
// a1 = (непрозрачность, вид: 0 капля, 1 пыль, 2 падающая масса). Три грани к камере (18 вершин), фрагменты - terrain.frag (брызги).
layout(location = 0) in vec4 aPos;
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
    gl_Position = viewProj * vec4(p - originOld.xyz, 1.0);   // Р69
    uint n = uint(ax * 2 + (plus ? 0 : 1));
    float light = 0.6 + 0.4 * max(dot(NORMALS[n], normalize(sunDir.xyz)), 0.0);
    // 0 капля, 1 пыль - белые; 2 падающая масса - вода
    vec3 c = aInfo.y > 1.5 ? vec3(0.3, 0.58, 0.85) : aInfo.y > 0.5 ? vec3(0.9, 0.94, 0.97) : vec3(0.86, 0.92, 0.97);
    vColor = vec4(c * light, aInfo.x);
    vWorld = p;
    vNormal = n;
    vWater = 3u;
    vFoam = 0.0;
}
