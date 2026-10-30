#version 460
// Р58: мелкая вода у камеры - столбики в тексель (1/16 блока) на блоках, где гладь не ровная (ступени, скаты) или
// вода быстрая. Экземпляр - блок (из списка), вершины - 256 столбиков × 30: верх (6) и 4 боковины (по 6) - боковина
// только вниз до соседнего столбика (или до его дна), внутренних стенок нет. Высота глади - плавно по центрам блоков
// (vb: у мокрого - верх воды, у сухого рядом с водой - вровень с низкой соседней водой или его земля), верх столбика
// - с шагом в тексель; волны - бегущий по течению шум, вниз, тоже в тексель. Мир - в блоках, камера - в прежних
// мерах (16 блоков, Р54).
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
layout(binding = 1) uniform sampler2D uFlow;   // скорость воды, блоки/с (по прежним блокам поля)
layout(binding = 4) uniform sampler2D uFine;   // r - vb (верх глади в центре блока, блоки; < -1e8 - нет), g - верх земли
layout(std430, binding = 3) readonly buffer Blocks { ivec2 blocks[]; };   // блоки с мелкой водой (мировые)
layout(location = 0) uniform ivec2 uOrigin;    // угол зоны в блоках (у текстуры - кайма в блок: индекс = b - origin + 1)

layout(location = 0) out vec4 vColor;
layout(location = 1) out vec3 vWorld;
layout(location = 2) flat out uint vNormal;
layout(location = 3) flat out uint vWater;

const vec3 WATER = vec3(0.3, 0.62, 0.92);
const vec3 NORMALS[6] = vec3[](vec3(1, 0, 0), vec3(-1, 0, 0), vec3(0, 1, 0), vec3(0, -1, 0), vec3(0, 0, 1), vec3(0, 0, -1));
const float NONE = -1e8;

uint ihash(ivec2 p) {
    uint h = uint(p.x) * 0x8DA6B343u ^ uint(p.y) * 0xD8163841u;
    h ^= h >> 13; h *= 0x5BD1E995u; h ^= h >> 15;
    return h;
}
float vnoise(vec2 q) {
    ivec2 i = ivec2(floor(q));
    vec2 f = q - vec2(i);
    f = f * f * (3.0 - 2.0 * f);
    float a = float(ihash(i) & 0xFFFFu) / 65535.0, b = float(ihash(i + ivec2(1, 0)) & 0xFFFFu) / 65535.0;
    float c = float(ihash(i + ivec2(0, 1)) & 0xFFFFu) / 65535.0, d = float(ihash(i + ivec2(1, 1)) & 0xFFFFu) / 65535.0;
    return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

vec2 fine(ivec2 b) { return texelFetch(uFine, b - uOrigin + 1, 0).rg; }

// верх глади в точке p (блоки, мир): по четырём центрам блоков; сухие без воды рядом - без веса
float surface(vec2 p) {
    vec2 q = p - 0.5;
    ivec2 i = ivec2(floor(q));
    vec2 f = q - vec2(i);
    float v[4] = float[](fine(i).r, fine(i + ivec2(1, 0)).r, fine(i + ivec2(0, 1)).r, fine(i + ivec2(1, 1)).r);
    float w[4] = float[]((1 - f.x) * (1 - f.y), f.x * (1 - f.y), (1 - f.x) * f.y, f.x * f.y);
    float sw = 0, s = 0;
    for (int k = 0; k < 4; ++k) if (v[k] > NONE) { sw += w[k]; s += w[k] * v[k]; }
    return sw > 0 ? s / sw : NONE;
}

// верх столбика текселя t (тексели, мир): гладь с шагом в тексель минус волна; NONE - сухо
float columnTop(ivec2 t, vec2 vel, float t8) {
    vec2 c = (vec2(t) + 0.5) / 16.0;
    float h = surface(c);
    if (h <= NONE) return NONE;
    float sp = length(vel);
    float amp = clamp((sp - 1.0) / 5.0, 0.0, 1.0) * 6.0;   // волна - до 6 текселей
    float n = vnoise(c * 0.9 - vel * t8 * 0.5) * 0.65 + vnoise(c * 2.3 + 3.1 - vel * t8 * 0.7) * 0.35;
    float top = floor((h - amp * n / 16.0) * 16.0 + 0.5) / 16.0;
    float ground = fine(ivec2(floor(c))).g;
    return top > ground + 0.5 / 16.0 ? top : NONE;
}

void main() {
    ivec2 b = blocks[gl_InstanceID];
    int col = gl_VertexID / 30, c = gl_VertexID % 30;
    ivec2 t = b * 16 + ivec2(col % 16, col / 16);   // тексель в мире
    // скорость - из поля течений (в прежних мерах), в блоках/с
    vec2 wc = (vec2(t) + 0.5) / 256.0;
    vec2 uv = (wc - flowArea.xy) / max(flowArea.z, 1.0);
    vec2 vel = all(greaterThanEqual(uv, vec2(0))) && all(lessThanEqual(uv, vec2(1))) ? textureLod(uFlow, uv, 0.0).xy : vec2(0);
    float t8 = floor(timeV.x * 8.0) / 8.0;
    float top = columnTop(t, vel, t8);
    vWater = 4u;
    vColor = vec4(0);
    vNormal = 2u;
    vWorld = vec3(0);
    if (top <= NONE) { gl_Position = vec4(0, 0, -2, 1); return; }   // сухо - вне экрана
    vec3 lo = vec3(float(t.x) / 16.0, 0, float(t.y) / 16.0);   // угол текселя в блоках
    const float S = 1.0 / 16.0;
    vec3 p;
    uint normal;
    // треугольники квада: 0-1-2, 0-2-3
    const int Q[6] = int[](0, 1, 2, 0, 2, 3);
    int k = Q[c % 6];
    if (c < 6) {
        normal = 2u;
        vec2 xz = vec2(k == 1 || k == 2 ? S : 0, k >= 2 ? S : 0);   // отсечение граней выключено - обход любой
        p = vec3(lo.x + xz.x, top, lo.z + xz.y);
    } else {
        int side = (c - 6) / 6;   // 0 +X, 1 -X, 2 +Z, 3 -Z
        ivec2 dn = side == 0 ? ivec2(1, 0) : side == 1 ? ivec2(-1, 0) : side == 2 ? ivec2(0, 1) : ivec2(0, -1);
        ivec2 tn = t + dn;
        float nt = columnTop(tn, vel, t8);
        float ng = fine(ivec2(floor((vec2(tn) + 0.5) / 16.0))).g;
        float low = max(nt > NONE ? nt : ng, ng);
        if (low >= top) { gl_Position = vec4(0, 0, -2, 1); return; }
        normal = side == 0 ? 0u : side == 1 ? 1u : side == 2 ? 4u : 5u;
        // грань на границе текселя к соседу: от low до top
        float x0 = lo.x + (side == 0 ? S : 0), z0 = lo.z + (side == 2 ? S : 0);
        bool alongZ = side < 2;
        float u = (k == 1 || k == 2) ? S : 0;
        float y = k >= 2 ? top : low;
        p = alongZ ? vec3(x0, y, lo.z + u) : vec3(lo.x + u, y, z0);
    }
    vec3 world = p / 16.0;   // в прежних мерах
    gl_Position = viewProj * vec4(world, 1.0);
    float light = 0.35 + 0.65 * max(dot(NORMALS[normal], normalize(sunDir.xyz)), 0.0);
    vColor = vec4(WATER * light, 0.55);
    vWorld = world;
    vNormal = normal;
}
