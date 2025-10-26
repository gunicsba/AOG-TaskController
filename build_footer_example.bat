@echo off
echo Building Console Footer Example...
echo ================================

REM Create a simple build directory
if not exist build_example mkdir build_example
cd build_example

REM Compile the example (assuming you have a C++ compiler like g++)
echo Compiling...
g++ -std=c++17 -I../include -I../lib ../src/console_footer.cpp ../src/footer_example.cpp -o footer_example.exe

if %ERRORLEVEL% EQU 0 (
    echo.
    echo Build successful!
    echo To run the example, execute: footer_example.exe
    echo.
) else (
    echo.
    echo Build failed!
    echo.
)

cd ..
pause