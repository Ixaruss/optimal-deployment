@echo off
setlocal EnableDelayedExpansion

:: ============================================================
::  CONFIGURE THESE PATHS FOR YOUR MACHINE
:: ============================================================
set GDAL_INC=C:\OSGeo4W\include
set GDAL_LIB=C:\OSGeo4W\lib
set GDAL_BIN=C:\OSGeo4W\bin

set JSON_INC=C:\jsoncpp\include
set JSON_LIB=C:\jsoncpp\lib

set MINGW_BIN=C:\mingw64\bin

set OUT_NAME=maplib
:: ============================================================

set CXX=%MINGW_BIN%\g++.exe
set CXXFLAGS=-std=c++17 -O2 -Wall ^
    -I. ^
    -I%GDAL_INC% ^
    -I%JSON_INC%

set LDFLAGS=-L%GDAL_LIB% -lgdal ^
    -L%JSON_LIB% -ljsoncpp ^
    -static-libgcc -static-libstdc++

:: output dirs
if not exist build mkdir build

echo.
echo [1/3] Compiling sources...
echo -------------------------------------------------------

set SRCS=^
    jsonParser.cc ^
    reproject.cc ^
    lib\elevation.cc ^
    lib\lib.cc ^
    lib\matrix.cc ^
    lib\query.cc ^
    lib\vizualise.cc ^
    exports.cc

set OBJS=
for %%f in (%SRCS%) do (
    :: flatten path for object file name  e.g. lib\matrix.cc → build\lib_matrix.o
    set "src=%%f"
    set "obj=%%f"
    set "obj=!obj:\=_!"
    set "obj=!obj:.cc=.o!"
    set OBJS=!OBJS! build\!obj!

    echo Compiling %%f ...
    %CXX% %CXXFLAGS% -DMAPLIB_EXPORTS -c %%f -o build\!obj!
    if errorlevel 1 (
        echo.
        echo [ERROR] Compilation failed on %%f
        goto :fail
    )
)

echo.
echo [2/3] Linking DLL...
echo -------------------------------------------------------
%CXX% -shared -o %OUT_NAME%.dll %OBJS% %LDFLAGS% ^
    -Wl,--out-implib,%OUT_NAME%.lib ^
    -Wl,--kill-at
if errorlevel 1 (
    echo.
    echo [ERROR] Link failed
    goto :fail
)

echo.
echo [3/3] Copying runtime DLLs...
echo -------------------------------------------------------
:: copy GDAL runtime so the DLL can find it at load time
copy /Y %GDAL_BIN%\gdal*.dll . >nul 2>&1
copy /Y %GDAL_BIN%\proj*.dll  . >nul 2>&1
copy /Y %GDAL_BIN%\geos*.dll  . >nul 2>&1

echo.
echo -------------------------------------------------------
echo  Build successful
echo    DLL  : %OUT_NAME%.dll
echo    LIB  : %OUT_NAME%.lib   (link stub for .NET/C++ projects)
echo    Place %OUT_NAME%.dll + gdal/proj/geos DLLs next to your .exe
echo -------------------------------------------------------
goto :end

:fail
echo.
echo Build FAILED. See errors above.
exit /b 1

:end
endlocal
