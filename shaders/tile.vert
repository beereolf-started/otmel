#version 460
// Узел плитки дальнего уровня (Р20): три числа - высоты (своя и уровня L+1), наклоны, покров.
// Положение по x и z - из номера узла в плитке; морфинг CDLOD - здесь же.
layout(location = 0) in uvec3 aData;

layout(std140, binding = 0) uniform Frame {
    mat4 viewProj;
    vec4 sunDir;
    vec4 eye;         // камера, блоки
    vec4 voxelZone;   // окно маски вокселей (Р21)
    vec4 fade;        // начало и конец растворения вдали
    vec4 sky;
};
layout(location = 0) uniform ivec2 uOrigin;   // начало плитки, блоки
layout(location = 1) uniform int uSpacing;    // шаг узлов, блоки
layout(location = 2) uniform vec2 uMorph;     // морфинг: начало и конец (дальность уровня)
layout(location = 3) uniform int uLevel;
layout(location = 4) uniform int uDebug;      // 1 - раскраска по уровням

layout(location = 0) out vec3 vColor;
layout(location = 1) out vec3 vWorld;

const vec3 COLORS[6] = vec3[](vec3(1, 0, 1), vec3(0.47, 0.47, 0.5), vec3(0.45, 0.32, 0.2), vec3(0.36, 0.58, 0.25), vec3(0.85, 0.8, 0.55), vec3(0.93, 0.95, 0.98));
const vec3 LEVEL_TINT[4] = vec3[](vec3(1.3, 0.8, 0.8), vec3(0.8, 1.3, 0.8), vec3(0.8, 0.8, 1.3), vec3(1.3, 1.3, 0.7));

float lightOf(vec3 n) { return 0.35 + 0.65 * max(dot(n, normalize(sunDir.xyz)), 0.0); }

// Как этот кусок склона выглядел бы вокселями (Р20). Ступенчатый склон с наклоном g (вокселей на воксель)
// на единицу площади - это 1 верхняя грань и |g.x|, |g.y| боковин вдоль x и z; боковины смотрят вниз по склону.
// Каждая грань входит своей ВИДИМОЙ площадью: площадь · max(0, нормаль · взгляд) - сколько точек экрана она
// заняла бы. Глядя вниз по склону, видны верхушки, вверх по склону - боковины, как у настоящих вокселей.
// Боковина ступеньки высотой |g|: верхний воксель - покров, остальное - слой под ним.
vec3 stairColor(vec2 g, uint top, uint under, vec3 view) {
    const vec3 up = vec3(0, 1, 0);
    vec3 ct = COLORS[min(top, 5u)], cu = COLORS[min(under, 5u)];
    float ax = abs(g.x), az = abs(g.y);
    vec3 nx = vec3(-sign(g.x), 0, 0), nz = vec3(0, 0, -sign(g.y));
    float wt = max(view.y, 0.0);
    float wx = ax * max(dot(nx, view), 0.0);
    float wz = az * max(dot(nz, view), 0.0);
    vec3 sideX = mix(cu, ct, min(1.0, 1.0 / max(ax, 1e-4)));
    vec3 sideZ = mix(cu, ct, min(1.0, 1.0 / max(az, 1e-4)));
    vec3 c = ct * lightOf(up) * wt + sideX * lightOf(nx) * wx + sideZ * lightOf(nz) * wz;
    float w = wt + wx + wz;
    return w > 1e-4 ? c / w : ct * lightOf(up);
}

void main() {
    uint i = uint(gl_VertexID) % 33u, j = uint(gl_VertexID) / 33u;
    // узел - центр столбца вокселей (+1/32 блока), как и у вокселей
    vec2 xz = vec2(uOrigin) + vec2(i, j) * float(uSpacing) + 1.0 / 32.0;
    // морфинг: по расстоянию до несдвинутого узла; нечётные узлы сползают к меньшему чётному соседу
    float k = clamp((distance(eye.xz, xz) - uMorph.x) / (uMorph.y - uMorph.x), 0.0, 1.0);
    xz -= vec2(i & 1u, j & 1u) * k * float(uSpacing);
    int h = bitfieldExtract(int(aData.x), 0, 16), hc = bitfieldExtract(int(aData.x), 16, 16);
    vec3 world = vec3(xz.x, (mix(float(h), float(hc), k) + 1.0) / 16.0, xz.y);   // верх верхнего вокселя

    vec2 g = vec2(bitfieldExtract(int(aData.y), 0, 8), bitfieldExtract(int(aData.y), 8, 8)) / 16.0;
    vec2 gc = vec2(bitfieldExtract(int(aData.y), 16, 8), bitfieldExtract(int(aData.y), 24, 8)) / 16.0;
    vec3 view = normalize(eye.xyz - world);
    vec3 cf = stairColor(g, aData.z & 15u, (aData.z >> 4) & 15u, view);
    vec3 cc = stairColor(gc, (aData.z >> 8) & 15u, (aData.z >> 12) & 15u, view);
    vColor = mix(cf, cc, k);
    if (uDebug == 1) vColor *= LEVEL_TINT[uLevel % 4];
    vWorld = world;
    gl_Position = viewProj * vec4(world, 1.0);
}
