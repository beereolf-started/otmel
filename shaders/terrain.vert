#version 460
// Вершина рельефа - одно 32-битное число (Р17): позиция в куске 3×6 бит, нормаль 3 бита, материал 11 бит.
layout(location = 0) in uint aPacked;

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
layout(location = 0) uniform ivec3 uChunk;   // начало куска в вокселях; у мелкого куска - блок-основа
layout(location = 1) uniform ivec3 uFineOff; // Р56: мелкий кусок - сдвиг от основы в мелких вокселях (1/16 блока)
layout(location = 2) uniform int uFine;      // Р56: 1 - мелкий кусок (дерево и прочее мельче блока)
layout(location = 3) uniform int uFineScale; // Р63: размер вокселя мелкого куска в мелких вокселях

layout(location = 0) out vec4 vColor;
layout(location = 1) out vec3 vWorld;          // Р43: точка в мире, блоки
layout(location = 2) flat out uint vNormal;    // Р43: 0 +X, 1 -X, 2 +Y, 3 -Y, 4 +Z, 5 -Z
layout(location = 3) flat out uint vWater;     // Р43: 1 - вода
layout(location = 4) out float vFoam;          // Р59: пена (мелкая вода у водопадов)

// цвета материалов, пока без текстур: 0 воздух, 1 камень, 2 земля, 3 трава, 4 песок, 5 снег, 6 вода (Р28)
// Р53: 7 кора, 8 древесина, 9 листва, 10 листва светлее
const vec3 COLORS[11] = vec3[](vec3(1, 0, 1), vec3(0.47, 0.47, 0.5), vec3(0.45, 0.32, 0.2), vec3(0.36, 0.58, 0.25), vec3(0.85, 0.8, 0.55), vec3(0.93, 0.95, 0.98), vec3(0.3, 0.62, 0.92),
                              vec3(0.31, 0.24, 0.17), vec3(0.68, 0.53, 0.33), vec3(0.22, 0.42, 0.14), vec3(0.32, 0.53, 0.2));
// Р34: вода полупрозрачная голубая - сквозь неё видно дно; та же доля - у дальних плиток (tile.vert)
const float WATER_ALPHA = 0.55;
const vec3 NORMALS[6] = vec3[](vec3(1, 0, 0), vec3(-1, 0, 0), vec3(0, 1, 0), vec3(0, -1, 0), vec3(0, 0, 1), vec3(0, 0, -1));

void main() {
    uint p = aPacked;
    vec3 local = vec3(p & 63u, (p >> 6) & 63u, (p >> 12) & 63u);
    uint normal = (p >> 18) & 7u;
    uint material = p >> 21;
    // мир - в прежних мерах (16 блоков мира, Р54); мелкий кусок - ещё в 16 раз мельче (воксель = тексель).
    // Р69: для видеокарты - относительно originV: разность считается в целых (точно), числа малые - вершины не дрожат
    // (у мира в сотни тысяч блоков у float шаг ~1/32 блока - мелкие воксели расходились щелями)
    ivec3 rc = uChunk - originV.xyz;
    vec3 rel = uFine == 1 ? (vec3(rc) + (local * float(uFineScale) + vec3(uFineOff)) / 16.0) / 16.0 : (local + vec3(rc)) / 16.0;
    // Р66: щели Т-стыков. Склеенные прямоугольники сходятся вершиной посреди чужого ребра - при растеризации остаются
    // щели меньше пикселя, сквозь них видно небо (на силуэтах деревьев, краях блоков). Непрозрачный прямоугольник
    // раздвигается за свои края на ~полпикселя на любом расстоянии; угол - по номеру вершины (4 на прямоугольник:
    // p0, p1, p2, p3 для «+», p0, p3, p2, p1 для «-»). Воду не трогаем - перекрытие прозрачного дало бы линии.
    if (material != 6u) {
        const ivec3 AX[3] = ivec3[](ivec3(0, 1, 2), ivec3(1, 2, 0), ivec3(2, 0, 1));   // (d, u, v): X - u=y, v=z ...
        int ai = int(normal) >> 1;
        bool minus = (normal & 1u) == 1u;
        int c = gl_VertexID & 3;
        int cu = minus ? int[](0, 0, 1, 1)[c] : int[](0, 1, 1, 0)[c];   // 1 - угол на дальнем по u крае
        int cv = minus ? int[](0, 1, 1, 0)[c] : int[](0, 0, 1, 1)[c];
        vec3 du = vec3(0), dv = vec3(0);
        du[AX[ai].y] = cu == 1 ? 1.0 : -1.0;
        dv[AX[ai].z] = cv == 1 ? 1.0 : -1.0;
        float eps = 0.0009 * distance(eye.xyz - originOld.xyz, rel);   // полпикселя при обзоре 70° на 720 строк
        rel += (du + dv) * eps;
    }
    gl_Position = viewProj * vec4(rel, 1.0);
    vec3 world = rel + originOld.xyz;   // для шума и расстояний - абсолютно
    // свет: рассеянный по нормали плюс постоянная подсветка неба
    float light = 0.35 + 0.65 * max(dot(NORMALS[normal], normalize(sunDir.xyz)), 0.0);
    vColor = vec4(COLORS[min(material, 10u)] * light, material == 6u ? WATER_ALPHA : 1.0);
    vWorld = world;
    vNormal = normal;
    vFoam = 0.0;
    vWater = material == 6u ? 1u : 0u;
}
