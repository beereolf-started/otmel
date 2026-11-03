#version 460
// Панель отладки поверх мира: один прямоугольник с готовой картинкой текста (dev_text.cpp).
// Вершины не передаются - прямоугольник строится из номера вершины, как это принято для накладок.
#ifdef VULKAN
layout(push_constant) uniform Push { vec4 rect; } pc;
#define uRect pc.rect
#define gl_VertexID gl_VertexIndex
#else
layout(location = 0) uniform vec4 uRect;   // левый, верхний, правый, нижний край в координатах экрана (-1..1)
#endif

layout(location = 0) out vec2 vUV;

void main() {
    // 0,1,2,2,1,3 - два треугольника прямоугольника
    const vec2 CORNER[4] = vec2[](vec2(0, 0), vec2(1, 0), vec2(0, 1), vec2(1, 1));
    const int ORDER[6] = int[](0, 1, 2, 2, 1, 3);
    vec2 c = CORNER[ORDER[gl_VertexID]];
    vUV = c;
    gl_Position = vec4(mix(uRect.x, uRect.z, c.x), mix(uRect.y, uRect.w, c.y), 0.0, 1.0);
}
