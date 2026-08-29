@echo off
rem Configure, build and run the host-bridge test suite in tests.
rem
rem Wire format, rate ladder, command line and log throttling - no serial port,
rem no shared memory, no board.
rem Needs CMake and the MSVC toolchain; doctest is fetched on first configure.
rem
rem Set NOPAUSE=1 to suppress the hold-open when calling this from a script.

setlocal enabledelayedexpansion

set "ROOT=%~dp0"
if "%ROOT:~-1%"=="\" set "ROOT=%ROOT:~0,-1%"
set "OUT=%ROOT%\build"

cmake -S "%ROOT%" -B "%OUT%" -DHOST_BRIDGE_BUILD_TESTS=ON
if errorlevel 1 (
  echo ERROR: cmake configure failed
  set EXITCODE=1
  goto :done
)

cmake --build "%OUT%" --config Release --target irTactileSerialBridge_tests
if errorlevel 1 (
  echo ERROR: build failed
  set EXITCODE=1
  goto :done
)

pushd "%OUT%"
ctest -C Release --output-on-failure
set EXITCODE=%errorlevel%
popd

:done
if %EXITCODE%==0 (echo TESTS PASSED) else (echo TESTS FAILED)
if "%NOPAUSE%"=="" echo %cmdcmdline% | "%SystemRoot%\System32\find.exe" /i "%~nx0" >nul && pause
exit /b %EXITCODE%
