@echo off
setlocal

echo ========================================
echo          PhysicsEngine Build
echo ========================================
echo.

REM ----------------------------------------
REM Find MSYS2 UCRT64
REM ----------------------------------------

if exist "C:\msys64\ucrt64\bin\g++.exe" (
    set "MSYS2_UCRT64=C:\msys64\ucrt64\bin"
) else (
    echo ERROR: MSYS2 UCRT64 was not found.
    echo.
    echo Please install MSYS2 and the UCRT64 toolchain.
    echo.
    echo Download:
    echo https://www.msys2.org/
    echo.
    pause
    exit /b 1
)

REM Add GCC and Ninja to PATH
set "PATH=%MSYS2_UCRT64%;%PATH%"

REM ----------------------------------------
REM Find CMake
REM ----------------------------------------

where cmake >nul 2>&1

if errorlevel 1 (
    echo ERROR: CMake was not found.
    echo.
    echo Please install CMake and add it to PATH.
    echo.
    echo Download:
    echo https://cmake.org/download/
    echo.
    pause
    exit /b 1
)

REM ----------------------------------------
REM Check compiler
REM ----------------------------------------

where g++ >nul 2>&1

if errorlevel 1 (
    echo ERROR: G++ was not found.
    echo.
    pause
    exit /b 1
)

REM ----------------------------------------
REM Check Ninja
REM ----------------------------------------

where ninja >nul 2>&1

if errorlevel 1 (
    echo ERROR: Ninja was not found.
    echo.
    pause
    exit /b 1
)

echo Tools found:
echo.
cmake --version
g++ --version
ninja --version

echo.
echo ========================================
echo [1/2] Configuring project
echo ========================================
echo.

cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release

if errorlevel 1 (
    echo.
    echo ERROR: CMake configuration failed.
    echo.
    pause
    exit /b 1
)

echo.
echo ========================================
echo [2/2] Building project
echo ========================================
echo.

cmake --build build

if errorlevel 1 (
    echo.
    echo ERROR: Build failed.
    echo.
    pause
    exit /b 1
)

echo.
echo ========================================
echo          BUILD SUCCESSFUL
echo ========================================
echo.
echo Executable:
echo %CD%\build\PhysicsEngine.exe
echo.

pause