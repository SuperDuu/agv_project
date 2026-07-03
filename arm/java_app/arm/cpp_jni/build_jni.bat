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

:: Try to find Visual Studio 2022 vcvars64.bat
set "VS_VCVARS=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

if exist "%VS_VCVARS%" (
    echo Found Visual Studio 2022 MSVC compiler!
    echo Setting up MSVC environment...
    call "%VS_VCVARS%" >nul
    
    echo Compiling with cl.exe...
    cl /LD /O2 /EHsc /I"%JAVA_HOME%\include" /I"%JAVA_HOME%\include\win32" kinematics_JniKinematics.cpp /Fe:..\lib\kinematics_jni.dll
    
    if %ERRORLEVEL% equ 0 (
        echo [SUCCESS] Compiled successfully to ../lib/kinematics_jni.dll
        :: Clean up temporary MSVC files
        if exist kinematics_JniKinematics.obj del kinematics_JniKinematics.obj
        if exist ..\lib\kinematics_jni.exp del ..\lib\kinematics_jni.exp
        if exist ..\lib\kinematics_jni.lib del ..\lib\kinematics_jni.lib
    ) else (
        echo [ERROR] MSVC compilation failed!
    )
) else (
    :: Fallback to g++
    echo Visual Studio MSVC compiler not found. Trying g++...
    g++ -shared -O3 -fPIC -I"%JAVA_HOME%\include" -I"%JAVA_HOME%\include\win32" kinematics_JniKinematics.cpp -o ../lib/kinematics_jni.dll
    
    if %ERRORLEVEL% equ 0 (
        echo [SUCCESS] Compiled successfully to ../lib/kinematics_jni.dll
    ) else (
        echo [ERROR] Compilation failed! Make sure g++ or Visual Studio is installed and in your PATH.
    )
)
pause
