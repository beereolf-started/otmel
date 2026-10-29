@echo off
rem Otmel build: MSVC environment, then CMake + Ninja.
rem   build.cmd          - debug build   (build\debug)
rem   build.cmd release  - fast build    (build\release)
rem cmd.exe: ASCII only and CRLF line endings, otherwise lines get split.
setlocal
set CFG=Debug
set DIR=build\debug
if /i "%1"=="release" (set CFG=RelWithDebInfo& set DIR=build\release)
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1
set "PATH=C:\Program Files\CMake\bin;%PATH%"
cd /d "%~dp0"
cmake -S . -B %DIR% -G Ninja -DCMAKE_BUILD_TYPE=%CFG% || exit /b 1
cmake --build %DIR% || exit /b 1
echo OK: %DIR%\otmel.exe
