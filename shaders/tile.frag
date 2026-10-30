#version 460
layout(location = 0) in vec4 vColor;
layout(location = 1) in vec3 vWorld;
layout(location = 2) in float vAbove;   // Р51: вода над своей землёй, воксели

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
