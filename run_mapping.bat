@echo off
chcp 65001 >nul
REM ============================================
REM mechdog_navigation 真机建图可视化启动脚本
REM ============================================
REM 用法: 双击运行, 或命令行带参数覆盖
REM   默认: 真机 + 建图可视化 + 静止位姿 + 采25帧存PGM
REM   传参: --sweep 360 旋转扫描; --frame-n 80 采80帧
REM
REM 前提: 相机已插好; OpenNI2\Drivers\orbbec.dll 已就位(见下)
REM ============================================

REM --- ① 加 SDK bin 到 PATH (否则找不到 astra_core.dll/orbbec.dll, 程序直接退出) ---
set "SDK=D:\orbbec ceram\AstraSDK-v2.1.3-94bca0f52e-20210608T034051Z-vs2015-win64"
set "PATH=%SDK%\bin;%PATH%"

REM --- ② 检查 OpenNI2 驱动(关键! 为什么相机有时打不开) ---
if exist "%SDK%\bin\OpenNI2\Drivers\orbbec.dll" (
    echo [OK] OpenNI2 驱动 orbbec.dll 已就位
) else (
    echo [WARN] OpenNI2\Drivers\orbbec.dll 缺失! 相机可能打不开(0x80070005)
    echo        修复: copy "%SDK%\bin\orbbec.dll" "%SDK%\bin\OpenNI2\Drivers\orbbec.dll"
)

REM --- ③ 切换到项目目录 & 启动 ---
cd /d "C:\Users\老w\Documents\dsh\mechdog_navigation"
echo.
echo 正在启动真机建图可视化... (窗口将弹出, 3栏: 彩色|点云|占据图+轨迹)
echo 静止模式采25帧后自动保存 mechdog_map_live.pgm
echo. 
echo 如需旋转扫描, 用: %~nx0 --sweep 360 --frame-n 80
echo ============================================
"%CD%\build_real_map\Release\mechdog_navigation.exe" --real --map --cloud %*
echo.
echo 程序已退出。地图文件: %CD%\mechdog_map_live.pgm
pause
