@echo off
REM One-click launcher: Astra Pro real-mode + point cloud visualization.
REM Close any previous viz window first - the camera is single-process;
REM a second instance gets an empty serial and no depth frames.
REM If the SDK moves, edit ASTRA_BIN below (keep this file ASCII-only:
REM cmd parses .bat in the ANSI codepage, UTF-8 Chinese comments break it).
set ASTRA_BIN=D:\orbbec ceram\AstraSDK-v2.1.3-94bca0f52e-20210608T034051Z-vs2015-win64\bin
set PATH=%ASTRA_BIN%;%PATH%
cd /d "%USERPROFILE%\.zcode\workspace\default\review-mechdog\mechdog_navigation\build_real\Release"
mechdog_navigation.exe --real --cloud
