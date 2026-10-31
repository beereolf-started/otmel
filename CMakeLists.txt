cmake_minimum_required(VERSION 3.25)
project(otmel LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Библиотеки подтягиваются при первой сборке (без отдельного менеджера пакетов)
include(FetchContent)
set(SDL_SHARED OFF CACHE BOOL "" FORCE)
set(SDL_STATIC ON CACHE BOOL "" FORCE)
set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
FetchContent_Declare(SDL3 GIT_REPOSITORY https://github.com/libsdl-org/SDL.git GIT_TAG release-3.2.0 GIT_SHALLOW TRUE)
FetchContent_Declare(glm GIT_REPOSITORY https://github.com/g-truc/glm.git GIT_TAG 1.0.1 GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(SDL3 glm)

# Загрузчик функций OpenGL 4.6 core (сгенерирован glad2, лежит в репозитории)
add_library(glad STATIC third_party/glad/src/gl.c)
target_include_directories(glad PUBLIC third_party/glad/include)

if(MSVC)
    add_compile_options(/utf-8 /W4 /permissive-)
endif()

# Генерация и мир - отдельной библиотекой: её же проверяют тесты, без окна и видеокарты
add_library(otmel_world STATIC
    src/gen/noise.cpp
    src/world/terrain.cpp
    src/world/mesher.cpp
    src/world/lod.cpp
    src/world/caves.cpp
    src/world/rivers.cpp
    src/world/flow.cpp
    src/world/water_sim.cpp
    src/world/spread.cpp
    src/world/oak.cpp
    src/world/forest.cpp
)
target_include_directories(otmel_world PUBLIC src)

add_executable(otmel
    src/main.cpp
    src/journal.cpp
    src/voxel_zone.cpp
    src/builder.cpp
    src/render/gl/gl_backend.cpp
)
target_include_directories(otmel PRIVATE src)
target_link_libraries(otmel PRIVATE otmel_world SDL3::SDL3-static glm::glm glad dbghelp)

# Шейдеры: GLSL -> SPIR-V при сборке (Р8), рядом с программой в папке shaders
file(GLOB GLSLC_CANDIDATES "C:/VulkanSDK/*/Bin/glslc.exe")
find_program(GLSLC glslc HINTS $ENV{VULKAN_SDK}/Bin ${GLSLC_CANDIDATES})
if(NOT GLSLC)
    list(GET GLSLC_CANDIDATES 0 GLSLC)
endif()
set(SHADERS terrain.vert terrain.frag tile.vert tile.frag dynwater.vert finewater.vert spray.vert treesprite.vert treesprite.frag)
set(SPV_FILES)
foreach(S ${SHADERS})
    set(OUT ${CMAKE_CURRENT_BINARY_DIR}/shaders/${S}.spv)
    add_custom_command(OUTPUT ${OUT}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/shaders
        COMMAND ${GLSLC} --target-env=opengl -O -o ${OUT} ${CMAKE_CURRENT_SOURCE_DIR}/shaders/${S}
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/shaders/${S}
        COMMENT "SPIR-V: ${S}")
    # Р73: та же копия под Vulkan (push-константы, gl_VertexIndex, ось Y) - имя.vk.spv
    set(OUTVK ${CMAKE_CURRENT_BINARY_DIR}/shaders/${S}.vk.spv)
    add_custom_command(OUTPUT ${OUTVK}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/shaders
        COMMAND ${GLSLC} --target-env=vulkan1.3 -O -o ${OUTVK} ${CMAKE_CURRENT_SOURCE_DIR}/shaders/${S}
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/shaders/${S}
        COMMENT "SPIR-V Vulkan: ${S}")
    list(APPEND SPV_FILES ${OUTVK})
    list(APPEND SPV_FILES ${OUT})
endforeach()
add_custom_target(otmel_shaders ALL DEPENDS ${SPV_FILES})
add_dependencies(otmel otmel_shaders)

# Карта мира сверху - тем же кодом генерации (Р19)
add_executable(otmel_map tools/biome_map.cpp)
target_link_libraries(otmel_map PRIVATE otmel_world)

# Тесты: то, что проверяется машиной (Р6)
enable_testing()
add_executable(otmel_tests tests/test_gen.cpp tests/mesher_reference.cpp)
target_link_libraries(otmel_tests PRIVATE otmel_world)
add_test(NAME generation COMMAND otmel_tests)
