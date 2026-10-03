@echo off
rem See scripts\open-editor.ps1 for options, e.g. OpenEditor.bat -EngineDir "D:\UE_5.5"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\open-editor.ps1" %*
if errorlevel 1 pause
