@echo off

setlocal

set "CarNDPathPlanningProjectPlatform=x64"
set "CarNDPathPlanningProjectToolset=v141"
set "CarNDPathPlanningProjectBuildType=Debug"

if NOT "%~1"=="" set "CarNDPathPlanningProjectPlatform=%~1"
if NOT "%~2"=="" set "CarNDPathPlanningProjectToolset=%~2"
if NOT "%~3"=="" set "CarNDPathPlanningProjectBuildType=%~3" 

set "VcPkgDir=c:\vcpkg"
set "VcPkgTriplet=%CarNDPathPlanningProjectPlatform%-windows"
rem set "VcPkgTriplet=%CarNDPathPlanningProjectPlatform%-windows-%CarNDPathPlanningProjectToolset%"

if defined VCPKG_ROOT_DIR if /i not "%VCPKG_ROOT_DIR%"=="" (
    set "VcPkgDir=%VCPKG_ROOT_DIR%"
)
if defined VCPKG_DEFAULT_TRIPLET if /i not "%VCPKG_DEFAULT_TRIPLET%"=="" (
    set "VcpkgTriplet=%VCPKG_DEFAULT_TRIPLET%"
)
set "VcPkgPath=%VcPkgDir%\vcpkg.exe"

echo. & echo Bootstrapping dependencies for triplet: %VcPkgTriplet% & echo.

rem ==============================
rem Update and Install packages
rem ==============================
call "%VcPkgPath%" update

rem Install latest uwebsockets
call "%VcPkgPath%" install uwebsockets --triplet %VcPkgTriplet%
rem Use adapted main.cpp for latest uwebsockets
rem copy main.cpp ..\..\src

rem ==============================
rem Configure CMake
rem ==============================

set "VcPkgTripletDir=%VcPkgDir%\installed\%VcPkgTriplet%"

set "CMAKE_PREFIX_PATH=%VcPkgTripletDir%;%CMAKE_PREFIX_PATH%"

echo. & echo Bootstrapping successful for triplet: %VcPkgTriplet% & echo CMAKE_PREFIX_PATH=%CMAKE_PREFIX_PATH% & echo.

set "CarNDPathPlanningProjectCMakeGeneratorName=Visual Studio 15 2017"

if "%CarNDPathPlanningProjectPlatform%"=="x86" (
    if "%CarNDPathPlanningProjectToolset%"=="v140" set "CarNDPathPlanningProjectCMakeGeneratorName=Visual Studio 14 2015"
    if "%CarNDPathPlanningProjectToolset%"=="v141" set "CarNDPathPlanningProjectCMakeGeneratorName=Visual Studio 15 2017"
)

if "%CarNDPathPlanningProjectPlatform%"=="x64" (
    if "%CarNDPathPlanningProjectToolset%"=="v140" set "CarNDPathPlanningProjectCMakeGeneratorName=Visual Studio 14 2015 Win64"
    if "%CarNDPathPlanningProjectToolset%"=="v141" set "CarNDPathPlanningProjectCMakeGeneratorName=Visual Studio 15 2017 Win64"
)

set "CarNDPathPlanningProjectBuildDir=%~dp0\..\..\products\cmake.msbuild.windows.%CarNDPathPlanningProjectPlatform%.%CarNDPathPlanningProjectToolset%"
if not exist "%CarNDPathPlanningProjectBuildDir%" mkdir "%CarNDPathPlanningProjectBuildDir%"
cd "%CarNDPathPlanningProjectBuildDir%"

echo: & echo CarNDPathPlanningProjectBuildDir=%CD% & echo cmake.exe -G "%CarNDPathPlanningProjectCMakeGeneratorName%" "%~dp0\..\.." & echo:

call cmake.exe -G "%CarNDPathPlanningProjectCMakeGeneratorName%" "%~dp0\..\.."

call "%VcPkgPath%" integrate install

endlocal