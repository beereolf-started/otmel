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

add_executable(otmel
    src/main.cpp
    src/journal.cpp
    src/render/gl/gl_backend.cpp
)
target_include_directories(otmel PRIVATE src)
# OpenGL - только у пути отрисовки (Р8): glad подключается лишь там, но линкуется к исполняемому файлу
target_link_libraries(otmel PRIVATE SDL3::SDL3-static glm::glm glad)
