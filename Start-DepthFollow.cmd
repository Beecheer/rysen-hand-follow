@echo off
where pwsh >nul 2>nul
if errorlevel 1 (echo PowerShell 7 is required. & pause & exit /b 1)
pwsh -NoProfile -File "%~dp0Start-DepthFollow.ps1" %*
set "depth_exit=%errorlevel%"
if not "%depth_exit%"=="0" pause
exit /b %depth_exit%
