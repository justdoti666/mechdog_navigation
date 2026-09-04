@echo off
chcp 65001 >nul
REM ============================================
REM mechdog_navigation 2.5D 近场地形可视化启动
REM ============================================
REM 用法: 双击运行(用默认外参) 或带参数:
REM   run_hm25.bat --pitch 10 --height 0.18    ← 相机装好后按实测填!
REM
REM 说明: 2.5D 依赖相机外参标定!
REM   --pitch  相机前俯角(度), 向下为+ (你要的约10°)
REM   --height 相机离地高度(米), 实测填 (约0.18)
REM 对着能看到地面+台阶/沟的场景, 2.5D会显示:
REM   绿=能走 橙=凸起 红=沟坑 蓝=坡陡 灰=未知
REM ============================================

REM --- ① 加 SDK bin 到 PATH ---
set "SDK=D:\orbbec ceram\AstraSDK-v2.1.3-94bca0f52e-20210608T034051Z-vs2015-win64"
set "PATH=%SDK%\bin;%PATH%"

REM --- ② 检查 OpenNI2 驱动 ---
if exist "%SDK%\bin\OpenNI2\Drivers\orbbec.dll" (
    echo [OK] OpenNI2 驱动 orbbec.dll 已就位
) else (
    echo [WARN] OpenNI2\Drivers\orbbec.dll 缺失! 相机可能打不开
    echo        修复: copy "%SDK%\bin\orbbec.dll" "%SDK%\bin\OpenNI2\Drivers\orbbec.dll"
)

REM --- ③ 启动 2.5D ---
cd /d "C:\Users\老w\Documents\dsh\mechdog_navigation"
echo.
echo [提醒] 2.5D 需要相机外参. 若画面全灰, 请加 --pitch ^<俯角^> --height ^<离地高^>
echo        例如: run_hm25.bat --pitch 10 --height 0.18
echo ============================================
"%CD%\build_real_map\Release\mechdog_navigation.exe" --real --hm25 --cloud %*
pause
