@echo off
setlocal
gcc "main.c" "lidar_io.c" "lidar_filter.c" "lidar_detect.c" "lidar_math.c" "lidar_draw.c" -o lidar ^
 -I"SDL3-3.2.24/x86_64-w64-mingw32/include" ^
 -L"SDL3-3.2.24/x86_64-w64-mingw32/lib" ^
 -I"SDL3_ttf-3.2.2/x86_64-w64-mingw32/include" ^
 -L"SDL3_ttf-3.2.2/x86_64-w64-mingw32/lib" ^
 -I"curl-8.16.0_8-win64-mingw/include" ^
 -L"curl-8.16.0_8-win64-mingw/lib" ^
 -lSDL3 -lSDL3_ttf -lcurl -lm

if %errorlevel% neq 0 (
  echo.
  echo Compilation failed!
) else (
  echo.
  echo Compiled!...
  lidar.exe
)

pause
