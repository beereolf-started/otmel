#version 460
// Панель отладки: показываем готовую картинку как есть, с её прозрачностью.
layout(location = 0) in vec2 vUV;
layout(location = 0) out vec4 outColor;
layout(binding = 0) uniform sampler2D uPanel;

void main() {
    outColor = texture(uPanel, vUV);
}
