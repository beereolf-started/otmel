#version 460
// Р65: рисунок спрайта дуба - процедурный, пиксельный (сетка 32×32 на спрайт): ствол снизу, крона - эллипс с
// неровным краем и просветами по шуму от семени, два оттенка листвы пятнами, сверху светлее. Цвета - как у дубов
// вблизи (terrain.vert: кора, листва). Дымка вдали - как у плиток.
layout(location = 0) in vec2 vUv;
layout(location = 1) in vec3 vWorld;
layout(location = 2) flat in float vSeed;
layout(location = 0) out vec4 outColor;

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

uint ihash(ivec3 p) {
    uint h = uint(p.x) * 0x8DA6B343u ^ uint(p.y) * 0xD8163841u ^ uint(p.z) * 0xCB1AB31Fu;
    h ^= h >> 13; h *= 0x5BD1E995u; h ^= h >> 15;
    return h;
}
float vnoise(vec2 q, int s) {
    ivec2 i = ivec2(floor(q));
    vec2 f = q - vec2(i);
    f = f * f * (3.0 - 2.0 * f);
    float a = float(ihash(ivec3(i, s)) & 0xFFFFu) / 65535.0, b = float(ihash(ivec3(i + ivec2(1, 0), s)) & 0xFFFFu) / 65535.0;
    float c = float(ihash(ivec3(i + ivec2(0, 1), s)) & 0xFFFFu) / 65535.0, d = float(ihash(ivec3(i + ivec2(1, 1), s)) & 0xFFFFu) / 65535.0;
    return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

void main() {
    int s = int(vSeed);
    vec2 px = (floor(vUv * 32.0) + 0.5) / 32.0;   // пиксели спрайта
    vec3 bark = vec3(0.31, 0.24, 0.17), leafA = vec3(0.22, 0.42, 0.14), leafB = vec3(0.32, 0.53, 0.2);
    vec3 col;
    // крона: эллипс, край и просветы - шумом
    vec2 cc = (px - vec2(0.5, 0.63)) / vec2(0.5, 0.38);
    float edge = 0.78 + 0.3 * vnoise(px * 7.0, s);
    float r = length(cc);
    if (r < edge && vnoise(px * 13.0, s + 7) > 0.22) {
        col = mix(leafA, leafB, step(0.5, vnoise(px * 5.0, s + 3)));
        // сверху светлее; в среднем - как объёмные дубы (у них тёмные боковые грани): ×0.5..0.8
        col *= 0.5 + 0.3 * clamp(cc.y * 0.5 + 0.5, 0.0, 1.0);
    } else if (abs(px.x - 0.5 - 0.03 * (vnoise(vec2(px.y * 4.0, 0.0), s + 5) - 0.5)) < 0.045 && px.y < 0.45) {
        col = bark * 0.7;
    } else {
        discard;
    }
    float d = distance(eye.xyz, vWorld);
    float haze = fade.w * (1.0 - exp(-d / fade.z));
    outColor = vec4(mix(col, sky.rgb, max(haze, smoothstep(fade.x, fade.y, d))), 1.0);
}
