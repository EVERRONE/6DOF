@echo off
rem ---------------------------------------------------------------------------
rem  Puts a "6DOF Robot Arm" shortcut on the Windows desktop that points at
rem  start-robot-arm.bat. Run this once; after that the app starts from the
rem  desktop icon, no command line involved.
rem ---------------------------------------------------------------------------

setlocal
cd /d "%~dp0"

if not exist "%~dp0start-robot-arm.bat" (
  echo.
  echo   start-robot-arm.bat was not found next to this file.
  echo   Keep both files together in the repository root.
  echo.
  pause
  exit /b 1
)

powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$link = Join-Path ([Environment]::GetFolderPath('Desktop')) '6DOF Robot Arm.lnk';" ^
  "$sc = (New-Object -ComObject WScript.Shell).CreateShortcut($link);" ^
  "$sc.TargetPath = '%~dp0start-robot-arm.bat';" ^
  "$sc.WorkingDirectory = '%~dp0';" ^
  "$sc.IconLocation = '%SystemRoot%\system32\shell32.dll,137';" ^
  "$sc.Description = 'Start the 6DOF robot arm control app';" ^
  "$sc.Save();" ^
  "Write-Host ('  Shortcut created: ' + $link)"

if errorlevel 1 (
  echo.
  echo   Creating the shortcut failed.
  echo   You can also right-click start-robot-arm.bat and pick
  echo   "Send to" - "Desktop (create shortcut)".
  echo.
  pause
  exit /b 1
)

echo.
echo   Done. Double-click "6DOF Robot Arm" on your desktop to start the app.
echo.
pause
endlocal
