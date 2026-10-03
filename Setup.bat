@echo off
rem One-time setup: builds the flight model and the Unreal project. Options: see scripts\setup.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\setup.ps1" %*
pause
