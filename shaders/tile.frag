#version 460
layout(location = 0) in vec4 vColor;
layout(location = 1) in vec3 vWorld;
layout(location = 2) in float vAbove;   // Р51: вода над своей землёй, воксели
layout(location = 3) in float vForest;  // Р67: густота леса 0..1

layout(std140, binding = 0) uniform Frame {
    mat4 viewProj;
    vec4 sunDir;
    vec4 eye;
    vec4 voxelZone;   // окно маски: minX, minZ (блоки), столбцов по стороне, блоков в столбце
    vec4 fade;
    vec4 sky;
    vec4 flowArea;
    vec4 timeV;
    vec4 simArea;
    mat4 invViewProj;  // Р51: из экрана в мир
};
layout(binding = 2) uniform sampler2D uSceneDepth;   // Р51: глубина сцены без воды (только в проходе воды)

layout(binding = 0) uniform usampler2D uVoxelMask;   // 1 - столбец вокселей готов

layout(location = 0) out vec4 outColor;

// Р67: шум крон - целочисленный хеш (мировые координаты - сотни тысяч блоков)
float chash(vec2 q) {
    uvec2 u = uvec2(ivec2(floor(q)));
    uint h = u.x * 0x8DA6B343u ^ u.y * 0xD8163841u;
    h ^= h >> 13; h *= 0x5BD1E995u; h ^= h >> 15;
    return float(h & 0xFFFFu) / 65535.0;
}
float cnoise(vec2 q) {
    vec2 i = floor(q), f = q - i;
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(chash(i), chash(i + vec2(1, 0)), f.x), mix(chash(i + vec2(0, 1)), chash(i + vec2(1, 1)), f.x), f.y);
}

void main() {
    // над готовыми столбцами вокселей гладкой поверхности нет: там рисуют воксели (Р21). Где столбец ещё
    // не построен - остаётся гладкий рельеф той же формы. Четверти плиток целиком над вокселями отброшены раньше.
    ivec2 c = ivec2(floor((vWorld.xz - voxelZone.xy) / voxelZone.w));
    int side = int(voxelZone.z);
    if (all(greaterThanEqual(c, ivec2(0))) && all(lessThan(c, ivec2(side))) && texelFetch(uVoxelMask, c, 0).r != 0u) discard;
    // воздушная дымка: даль голубеет (1 - e^(-d/L)), но видна - она и даёт ощущение расстояния;
    // у края видимости рельеф растворяется в небе целиком - конец мира не обрывается стеной
    float d = distance(eye.xyz, vWorld);
    float haze = fade.w * (1.0 - exp(-d / fade.z));
    vec3 col = vColor.rgb;
    float a = vColor.a;
    // Р67: лес вдали - пятна крон на земле, где по формуле густоты должен быть лес (как у дубов и спрайтов);
    // начинается там, где кончаются спрайты (Р68: 3.6 км); где пятно меньше пикселя - средний тон
    if (a >= 0.99 && vForest > 0.01) {
        float k = smoothstep(215.0, 225.0, d);   // Р68: спрайты - до 3.6 км (225 прежних блоков)
        if (k > 0.0) {
            vec2 wb = vWorld.xz * 16.0;   // блоки
            float n = cnoise(wb / 6.0) * 0.6 + cnoise(wb / 2.7 + 5.3) * 0.4;
            float cover = min(1.0, vForest * 1.05);
            float crown = smoothstep(1.0 - cover - 0.12, 1.0 - cover + 0.12, n);
            float px = max(fwidth(wb.x), fwidth(wb.y)) / 6.0;   // крон на пиксель
            crown = mix(crown, cover, clamp(px * 1.5 - 0.5, 0.0, 1.0));
            vec3 canopy = mix(vec3(0.14, 0.27, 0.09), vec3(0.2, 0.36, 0.13), cnoise(wb / 3.1 + 11.0));
            col = mix(col, canopy, crown * k);
        }
    }
    if (a < 0.99) {   // вода (0.55); у земли 1.0 после интерполяции бывает 0.99999994 - земля красилась крапинкой воды
        // Р51: вода ниже своей земли (сухие узлы держат её под дном) не рисуется: в щели у края вокселей она
        // просвечивала светлой линией
        if (vAbove < 0.0) discard;
        // Р51: вода вдали - так же, как у вокселей (terrain.frag): поглощение по толщине воды по лучу
        vec2 suv = gl_FragCoord.xy / vec2(textureSize(uSceneDepth, 0));
        float sceneZ = texture(uSceneDepth, suv).r;
        float thick = 64.0;
        if (sceneZ > 0.0) {
            vec4 p = invViewProj * vec4(suv * 2.0 - 1.0, sceneZ, 1.0);
            thick = max(0.0, distance(p.xyz / p.w, eye.xyz) - d);
        }
        float absorb = 1.0 - exp(-thick * 0.32);
        col = mix(col * vec3(0.55, 0.72, 0.8), vec3(0.015, 0.07, 0.10), absorb);
        a = mix(0.18, 0.96, absorb);
    }
    outColor = vec4(mix(col, sky.rgb, max(haze, smoothstep(fade.x, fade.y, d))), a);
}
