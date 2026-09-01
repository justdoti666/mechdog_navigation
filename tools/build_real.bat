@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul 2>&1
cd /d "%~dp0.."
cmake -B build_real -DUSE_ASTRA_SDK=ON "-DASTRA_SDK_ROOT=D:/orbbec ceram/AstraSDK-v2.1.3-94bca0f52e-20210608T034051Z-vs2015-win64"
if errorlevel 1 exit /b 1
cmake --build build_real --config Release --target mapping_real_test
if errorlevel 1 exit /b 1
echo BUILD_OK
