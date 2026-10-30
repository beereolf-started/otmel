#version 460
layout(location = 0) in vec4 vColor;
layout(location = 0) out vec4 outColor;

void main() {
    outColor = vColor;   // Р34: у воды - полупрозрачность, у грунта - 1
}
