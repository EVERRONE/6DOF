@echo off
rem ---------------------------------------------------------------------------
rem  6DOF robot arm -- start the control app (Windows)
rem
rem  Double-click this file. It pulls the latest commits, installs any changed
rem  dependencies, starts the web app and opens it in Chrome or Edge.
rem
rem  Options (also work when dragged onto this file's shortcut):
rem    --no-update     start without pulling from GitHub
rem    --update-only   only pull + install, do not start the app
rem ---------------------------------------------------------------------------

setlocal
title 6DOF Robot Arm
cd /d "%~dp0"

where node >nul 2>nul
if errorlevel 1 (
  echo.
  echo   Node.js was not found.
  echo.
  echo   Install the LTS version from https://nodejs.org/ , then
  echo   double-click this file again.
  echo.
  pause
  exit /b 1
)

node "robot-arm-control\scripts\launch.mjs" %*
set "EXITCODE=%ERRORLEVEL%"

if not "%EXITCODE%"=="0" (
  echo.
  echo   The launcher stopped with error code %EXITCODE%.
  echo.
  pause
)

endlocal & exit /b %EXITCODE%
