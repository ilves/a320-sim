@echo off
rem See scripts\play.ps1 for options, e.g. Play.bat -EngineDir "D:\UE_5.5"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\play.ps1" %*
if errorlevel 1 pause
