@echo off
call "%~dp0Stop-DepthCamera.cmd" %*
exit /b %errorlevel%
