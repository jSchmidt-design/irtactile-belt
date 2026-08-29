@echo off
rem Compile the firmware with arduino-cli.
rem
rem   compile.bat                  build (incremental - fast retries)
rem   compile.bat --clean          full rebuild
rem   compile.bat --warnings all   build with all compiler warnings
rem   compile.bat -u -p COM6       build and upload
rem
rem Any extra arguments are passed straight through to "arduino-cli compile".
rem
rem On failure the window is held open so the error stays readable. Set
rem NOPAUSE=1 to suppress that when calling this from another script.

setlocal

set FQBN=esp32:esp32:esp32
set "BUILD=%~dp0build"

set "SKETCH=%~dp0src\irtactile-belt"
if "%SKETCH:~-1%"=="\" set "SKETCH=%SKETCH:~0,-1%"

rem Prefer arduino-cli on PATH, otherwise the one bundled with the Arduino IDE.
set CLI=
for /f "delims=" %%I in ('where arduino-cli 2^>nul') do if not defined CLI set "CLI=%%I"
if not defined CLI call :try "%ProgramFiles%\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
if not defined CLI call :try "%ProgramFiles(x86)%\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
if not defined CLI call :try "%LOCALAPPDATA%\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"

if not defined CLI (
  echo ERROR: arduino-cli not found.
  echo Install the Arduino IDE, or put arduino-cli.exe on your PATH.
  set EXITCODE=1
  goto :done
)

rem Use the IDE's own package/library directories so the esp32 core is found.
set ARDUINO_DIRECTORIES_DATA=%LOCALAPPDATA%\Arduino15
set ARDUINO_DIRECTORIES_DOWNLOADS=%LOCALAPPDATA%\Arduino15\staging
set ARDUINO_DIRECTORIES_USER=%USERPROFILE%\Documents\Arduino

echo Using   : %CLI%
echo Board   : %FQBN%
echo Sketch  : %SKETCH%
echo.

"%CLI%" compile --fqbn %FQBN% --build-path "%BUILD%" %* "%SKETCH%"
set EXITCODE=%ERRORLEVEL%

echo.
if %EXITCODE%==0 (echo BUILD OK) else (echo BUILD FAILED - exit code %EXITCODE%)

:done
rem Keep the window open when launched by double-click, not when run from a
rem shell. Use the full path to find.exe - a Unix find on PATH (Git, MSYS)
rem would shadow it.
if "%NOPAUSE%"=="" echo %cmdcmdline% | "%SystemRoot%\System32\find.exe" /i "%~nx0" >nul && pause
exit /b %EXITCODE%

:try
if exist %1 set "CLI=%~1"
goto :eof
