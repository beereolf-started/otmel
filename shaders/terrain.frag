#version 460
layout(location = 0) in vec4 vColor;
layout(location = 1) in vec3 vWorld;
layout(location = 2) flat in uint vNormal;
layout(location = 3) flat in uint vWater;
layout(location = 0) out vec4 outColor;

layout(std140, binding = 0) uniform Frame {
    mat4 viewProj;
    vec4 sunDir;
    vec4 eye;
    vec4 voxelZone;
    vec4 fade;
    vec4 sky;
    vec4 flowArea;     // Р43: поле течений - угол x, z, сторона (блоки)
    vec4 timeV;        // Р43: x - секунды
    vec4 simArea;      // Р45: зона автомата воды - угол x, z, сторона (блоки), w - 1, если есть
    mat4 invViewProj;  // Р51: из экрана в мир
};
layout(binding = 1) uniform sampler2D uFlow;         // Р43: скорость воды, блоки/с (x, z)
layout(binding = 2) uniform sampler2D uSceneDepth;   // Р51: глубина сцены без воды

// шум для ряби: значения в узлах решётки, между ними - плавно
float hash(vec2 q) { return fract(sin(dot(q, vec2(127.1, 311.7))) * 43758.5453); }
float valueNoise(vec2 q) {
    vec2 i = floor(q), f = fract(q);
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash(i), hash(i + vec2(1, 0)), f.x), mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), f.x), f.y);
}
float ripples(vec2 q) { return valueNoise(q) * 0.6 + valueNoise(q * 2.3 + 7.1) * 0.4; }

// Р51: вода поглощает свет по толщине (закон Бугера): мелко - почти прозрачно и светло, глубоко - темно и мутно
const vec3 DEEP = vec3(0.015, 0.07, 0.10);
const float ABSORB = 0.32;   // на блок толщины по лучу

void main() {
    if (vWater == 0u) { outColor = vColor; return; }   // грунт - как был
    // Р51: пиксельно - рябь, пена и полосы считаются по сетке вокселя (1/16 блока), время - кадрами по 1/8 с:
    // блик перескакивает с вокселя на воксель, а не плывёт
    // (точка сдвинута на полвокселя внутрь грани - грань лежит ровно на границе вокселей, без дрожи округления)
    const vec3 NORMALS[6] = vec3[](vec3(1, 0, 0), vec3(-1, 0, 0), vec3(0, 1, 0), vec3(0, -1, 0), vec3(0, 0, 1), vec3(0, 0, -1));
    vec3 px = (floor(vWorld * 16.0 - NORMALS[min(vNormal, 5u)] * 0.5) + 0.5) / 16.0;
    float t = floor(timeV.x * 8.0) / 8.0;
    // Р43: скорость воды здесь - из поля течений (вне поля - стоячая)
    vec2 uv = (px.xz - flowArea.xy) / max(flowArea.z, 1.0);
    vec2 v = all(greaterThanEqual(uv, vec2(0))) && all(lessThanEqual(uv, vec2(1))) ? texture(uFlow, uv).xy : vec2(0);
    float speed = length(v);
    // толщина воды по лучу: от глади до того, что за ней (дна); неба за водой нет - считаем глубоко
    vec2 suv = gl_FragCoord.xy / vec2(textureSize(uSceneDepth, 0));
    float sceneZ = texture(uSceneDepth, suv).r;
    float thick = 64.0;
    if (sceneZ > 0.0) {
        vec4 p = invViewProj * vec4(suv * 2.0 - 1.0, sceneZ, 1.0);
        thick = max(0.0, distance(p.xyz / p.w, eye.xyz) - distance(vWorld, eye.xyz));
    }
    float absorb = 1.0 - exp(-thick * ABSORB);
    vec3 c = mix(vColor.rgb * vec3(0.55, 0.72, 0.8), DEEP, absorb);
    float a = mix(0.18, 0.96, absorb);
    if (vNormal == 2u) {
        // гладь: рябь сносится течением. Две фазы по 4 с со сдвигом на полпериода, вес - треугольник: рисунок
        // уезжает по течению и незаметно сменяется, не растягиваясь бесконечно
        float ph0 = fract(t * 0.25), ph1 = fract(t * 0.25 + 0.5);
        float w0 = 1.0 - abs(2.0 * ph0 - 1.0);
        vec2 q = px.xz * 0.5;
        float n0 = ripples(q - v * ph0 * 2.0);
        float n1 = ripples(q - v * ph1 * 2.0 + 0.37);
        float n = mix(n1, n0, w0);
        // в стоячей воде - медленный дрейф ряби (ветер)
        n = mix(n, ripples(q * 0.8 + vec2(t * 0.05, t * 0.03)), clamp(1.0 - speed * 2.0, 0.0, 1.0));
        c *= 0.85 + 0.3 * n;
        // блик неба - на гребнях ряби, ступенькой (пиксель горит или нет)
        c += vec3(0.16, 0.2, 0.22) * step(0.78, n);
        // пена - на быстрых местах (перекаты, пороги): скорость от 2 до 6 блоков/с
        float foam = smoothstep(2.0, 6.0, speed) * step(0.62, n);
        c = mix(c, vec3(0.9, 0.95, 1.0), foam);
        a = mix(a, 0.92, foam);
    } else if (vNormal != 3u) {
        // боковые грани воды - ступени перекатов и обрывы: падающая вода, полосы бегут вниз
        float s = valueNoise(vec2((px.x + px.z) * 3.0, px.y * 1.5 + t * 6.0));
        c = mix(c * 1.15, vec3(0.88, 0.94, 1.0), step(0.68, s) * 0.8);
        a = max(a, 0.7);
    }
    outColor = vec4(c, a);
}
