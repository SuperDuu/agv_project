@echo off
echo ===================================================
echo Compiling C++ JNI library (kinematics_jni.dll)...
echo ===================================================

if "%JAVA_HOME%"=="" (
    echo [ERROR] JAVA_HOME environment variable is not set!
    echo Please set JAVA_HOME to your JDK installation path (e.g. C:\Program Files\Java\jdk-17)
    pause
    exit /b 1
)

echo Found JAVA_HOME: %JAVA_HOME%

g++ -shared -O3 -fPIC -I"%JAVA_HOME%\include" -I"%JAVA_HOME%\include\win32" kinematics_JniKinematics.cpp -o ../lib/kinematics_jni.dll

if %ERRORLEVEL% equ 0 (
    echo [SUCCESS] Compiled successfully to ../lib/kinematics_jni.dll
) else (
    echo [ERROR] Compilation failed! Make sure g++ is installed and in your PATH.
)
pause
