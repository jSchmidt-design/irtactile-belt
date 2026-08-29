@echo off
rem Build and run the host-side tests in tests\.
rem
rem These compile the firmware's own headers (SerialDecoder.h, SampleRing.h,
rem Playout.h, WrapTracker.h) natively, so they need MSVC - not arduino-cli.
rem Nothing is flashed and no board is involved.
rem
rem   test.bat            build and run all suites
rem   test.bat decoder    only the decoder suite
rem   test.bat playout    only the playout suite
rem   test.bat counter    only the counter suite
rem
rem Set NOPAUSE=1 to suppress the hold-open when calling this from a script.

setlocal enabledelayedexpansion

set "ROOT=%~dp0"
if "%ROOT:~-1%"=="\" set "ROOT=%ROOT:~0,-1%"
set "OUT=%ROOT%\tests\build"

set SUITES=decoder playout counter
if not "%~1"=="" set SUITES=%~1

rem Set up the MSVC environment unless we are already inside a dev prompt.
if defined VCINSTALLDIR goto :havevc
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" set "VSWHERE=%ProgramFiles%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
  echo ERROR: vswhere.exe not found - is Visual Studio installed?
  set EXITCODE=1
  goto :done
)
set "VSPATH="
for /f "usebackq delims=" %%I in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSPATH=%%I"
if not defined VSPATH (
  echo ERROR: no Visual Studio install with the C++ toolset was found.
  set EXITCODE=1
  goto :done
)
call "%VSPATH%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 (
  echo ERROR: could not initialise the MSVC environment.
  set EXITCODE=1
  goto :done
)
:havevc

if not exist "%OUT%" mkdir "%OUT%"
set EXITCODE=0

for %%S in (%SUITES%) do (
  if not exist "%ROOT%\tests\%%S_test.cpp" (
    echo ERROR: no such suite: %%S
    set EXITCODE=1
  ) else (
    echo === %%S ===
    cl /nologo /EHsc /W3 /O2 /std:c++17 /D_USE_MATH_DEFINES ^
       /I "%ROOT%\src\irtactile-belt" ^
       "%ROOT%\tests\%%S_test.cpp" ^
       /Fo"%OUT%\\" /Fe"%OUT%\%%S_test.exe" >nul
    if errorlevel 1 (
      echo COMPILE FAILED
      set EXITCODE=1
    ) else (
      "%OUT%\%%S_test.exe"
      if errorlevel 1 set EXITCODE=1
    )
    echo.
  )
)

if %EXITCODE%==0 (echo TESTS PASSED) else (echo TESTS FAILED)

:done
if "%NOPAUSE%"=="" echo %cmdcmdline% | "%SystemRoot%\System32\find.exe" /i "%~nx0" >nul && pause
exit /b %EXITCODE%
