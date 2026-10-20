@echo off
pwsh -NoProfile -ExecutionPolicy Bypass -File "%~dp0Stop-HandCompare.ps1"
if errorlevel 1 pause
