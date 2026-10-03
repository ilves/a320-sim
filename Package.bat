@echo off
rem See scripts\package.ps1 for options, e.g. Package.bat -EngineDir "D:\UE_5.5"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\package.ps1" %*
if errorlevel 1 pause
