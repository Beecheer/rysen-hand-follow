@echo off
where pwsh >nul 2>nul
if errorlevel 1 (
  echo PowerShell 7 ^(pwsh^) is required. See README.md.
  pause
  exit /b 1
)
pwsh -NoProfile -File "%~dp0Stop-HandFollow.ps1" %*
set "handfollow_exit=%errorlevel%"
if not "%handfollow_exit%"=="0" pause
exit /b %handfollow_exit%
