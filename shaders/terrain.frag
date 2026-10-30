#version 460
layout(location = 0) in vec4 vColor;
layout(location = 1) in vec3 vWorld;
layout(location = 2) flat in uint vNormal;
layout(location = 3) flat in uint vWater;
layout(location = 4) in float vFoam;   // Р59: пена мелкой воды
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
layout(binding = 5) uniform usampler2D uFineMask;   // Р58: 1 - блок рисуется мелкой водой (зона - simArea)

// шум для ряби: значения в узлах решётки, между ними - плавно
// Р55: хеш - целочисленный: sin от мировых координат (сотни тысяч блоков, Р54) теряет точность - рябь
// превращалась в «снег» по текселю
float hash(vec2 q) {
    uvec2 u = uvec2(ivec2(floor(q)));
    uint h = u.x * 0x8DA6B343u ^ u.y * 0xD8163841u;
    h ^= h >> 13; h *= 0x5BD1E995u; h ^= h >> 15;
    return float(h & 0xFFFFFFu) / 16777215.0;
}
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
    if (vWater == 3u) { outColor = vColor; return; }   // Р52: брызги - белые воксели как есть
    // Р55 (Р54: воксель = блок мира): вода детальнее блока - рябь, блики, пена и полосы считаются по сетке
    // ТЕКСЕЛЕЙ (1/16 блока), время - кадрами по 1/8 с: блик перескакивает с текселя на тексель. wb - точка в блоках
    // мира; точка сдвинута на полтекселя внутрь грани (грань - ровно на границе, без дрожи округления)
    const vec3 NORMALS[6] = vec3[](vec3(1, 0, 0), vec3(-1, 0, 0), vec3(0, 1, 0), vec3(0, -1, 0), vec3(0, 0, 1), vec3(0, 0, -1));
    vec3 wb = vWorld * 16.0;
    vec3 px = (floor(wb * 16.0 - NORMALS[min(vNormal, 5u)] * 0.5) + 0.5) / 16.0;
    float t = floor(timeV.x * 8.0) / 8.0;
    // вдали тексель меньше пикселя - мелочь гаснет к среднему (иначе мерцает при движении)
    // Р58: вода кусков там, где рисуется мелкая вода (столбики в тексель), - не рисуется
    if (vWater == 1u && simArea.w > 0.0) {
        ivec2 b = ivec2(floor(px.xz - simArea.xy * 16.0));
        int side = int(simArea.z * 16.0 + 0.5);
        if (all(greaterThanEqual(b, ivec2(0))) && all(lessThan(b, ivec2(side))) && texelFetch(uFineMask, b, 0).r != 0u) discard;
    }
    float tpp = max(fwidth(wb.x), fwidth(wb.z)) * 16.0;   // текселей на пиксель
    float detail = clamp(1.5 - tpp, 0.0, 1.0);
    // Р43: скорость воды здесь - из поля течений (вне поля - стоячая); блоки/с = м/с
    vec2 uv = (vWorld.xz - flowArea.xy) / max(flowArea.z, 1.0);
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
        vec2 q = px.xz * 0.7;
        float n0 = ripples(q - v * ph0 * 2.0);
        float n1 = ripples(q - v * ph1 * 2.0 + 0.37);
        float n = mix(n1, n0, w0);
        // в стоячей воде - медленный дрейф ряби (ветер)
        n = mix(n, ripples(q * 0.8 + vec2(t * 0.3, t * 0.2)), clamp(1.0 - speed * 2.0, 0.0, 1.0));
        n = mix(0.5, n, detail);
        c *= 0.85 + 0.3 * n;
        // блик неба - на гребнях ряби, ступенькой (тексель горит или нет)
        c += vec3(0.16, 0.2, 0.22) * step(0.78, n) * detail;
        // пена - на быстрых местах (перекаты, пороги): скорость от 2 до 6 блоков/с
        float foam = smoothstep(2.0, 6.0, speed) * mix(0.3, step(0.62, n), detail);
        // Р59: у водопадов (кромка, место падения) - бурлящая пена: крупные клочья поверх мелких, бегут и
        // перемешиваются по кадрам
        if (vWater == 4u && vFoam > 0.0) {
            float churn = ripples(px.xz * 1.6 + vec2(t * 1.1, -t * 0.7)) * 0.6 + ripples(px.xz * 4.1 - vec2(t * 0.9, t * 1.3)) * 0.4;
            foam = max(foam, step(1.0 - 0.85 * vFoam, churn) * mix(0.6, 1.0, detail));
        }
        c = mix(c, vec3(0.9, 0.95, 1.0), foam);
        a = mix(a, 0.92, foam);
    } else if (vNormal != 3u) {
        // боковые грани воды - ступени перекатов и обрывы: падающая вода, полосы бегут вниз
        float s = valueNoise(vec2((px.x + px.z) * 2.0, px.y * 1.0 + t * 8.0));
        c = mix(c * 1.15, vec3(0.88, 0.94, 1.0), mix(0.25, step(0.68, s), detail) * 0.8);
        a = max(a, 0.7);
    }
    outColor = vec4(c, a);
}
