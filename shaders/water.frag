#version 460
layout(location = 0) in vec3 vWorld;

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
    // глубокая синева; под пологим взглядом вода больше отражает небо (грубо, по Френелю: 1 - cos^5)
    vec3 v = normalize(eye.xyz - vWorld);
    float fres = pow(1.0 - abs(v.y), 5.0);
    vec3 c = mix(vec3(0.10, 0.27, 0.42), sky.rgb, 0.15 + 0.6 * fres);
    // та же дымка и растворение у края видимости, что у рельефа
    float d = distance(eye.xyz, vWorld);
    float haze = fade.w * (1.0 - exp(-d / fade.z));
    c = mix(c, sky.rgb, max(haze, smoothstep(fade.x, fade.y, d)));
    outColor = vec4(c, mix(0.78, 1.0, fres));
}
