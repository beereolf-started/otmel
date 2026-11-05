#version 460
// Панель отладки: показываем готовую картинку как есть, с её прозрачностью.
layout(location = 0) in vec2 vUV;
layout(location = 0) out vec4 outColor;
#ifdef VULKAN   // Р99: в Vulkan привязки общие с миром - картинка на девятой
layout(binding = 9) uniform sampler2D uPanel;
#else
layout(binding = 0) uniform sampler2D uPanel;
#endif

void main() {
    outColor = texture(uPanel, vUV);
}
