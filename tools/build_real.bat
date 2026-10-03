@echo off
REM B16 (2026-10-03): locate MSVC via vswhere; fallback to known BuildTools path.
REM This script builds mapping_real_test into build_real\ (100-frame real-machine
REM validation tool). Main GUI app -> build_real_map\ (docs/REAL_MAPPING_GUIDE.md).
set "VCVARS="
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" set "VSWHERE="
if defined VSWHERE for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VCVARS=%%i\VC\Auxiliary\Build\vcvarsall.bat"
if not defined VCVARS set "VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvarsall.bat"
if not exist "%VCVARS%" goto :no_vcvars
echo [env] vcvarsall: "%VCVARS%"
call "%VCVARS%" x64
if errorlevel 1 goto :vcvars_failed
goto :after_env
:no_vcvars
echo [ERR] vcvarsall.bat not found: "%VCVARS%"
echo       Install VS Build Tools (C++ toolchain) or edit VCVARS here.
exit /b 1
:vcvars_failed
echo [ERR] failed to init VS environment
exit /b 1
:after_env
cd /d "%~dp0.."
cmake -B build_real -DUSE_ASTRA_SDK=ON "-DASTRA_SDK_ROOT=D:/orbbec ceram/AstraSDK-v2.1.3-94bca0f52e-20210608T034051Z-vs2015-win64"
if errorlevel 1 exit /b 1
cmake --build build_real --config Release --target mapping_real_test
if errorlevel 1 exit /b 1
echo BUILD_OK
