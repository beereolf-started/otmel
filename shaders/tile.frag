#version 460
layout(location = 0) in vec3 vColor;
layout(location = 1) in vec3 vWorld;

layout(std140, binding = 0) uniform Frame {
    mat4 viewProj;
    vec4 sunDir;
    vec4 eye;
    vec4 voxelZone;
    vec4 fade;
    vec4 sky;
};

layout(location = 0) out vec4 outColor;

void main() {
    // над зоной вокселей гладкой поверхности нет: там рисуют воксели (плитки целиком внутри отброшены раньше)
    if (vWorld.x > voxelZone.x && vWorld.x < voxelZone.z && vWorld.z > voxelZone.y && vWorld.z < voxelZone.w) discard;
    // у края видимости рельеф растворяется в небе - конец мира не обрывается стеной
    float d = distance(eye.xz, vWorld.xz);
    outColor = vec4(mix(vColor, sky.rgb, smoothstep(fade.x, fade.y, d)), 1.0);
}
