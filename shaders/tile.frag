#version 460
layout(location = 0) in vec3 vColor;
layout(location = 1) in vec3 vWorld;

layout(std140, binding = 0) uniform Frame {
    mat4 viewProj;
    vec4 sunDir;
    vec4 eye;
    vec4 voxelZone;   // окно маски: minX, minZ (блоки), столбцов по стороне, блоков в столбце
    vec4 fade;
    vec4 sky;
};

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
    outColor = vec4(mix(vColor, sky.rgb, max(haze, smoothstep(fade.x, fade.y, d))), 1.0);
}
