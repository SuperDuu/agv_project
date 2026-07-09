@echo off
setlocal

cd /d "%~dp0"

set "JAVA_CMD=java"
set "JAVAC_CMD=javac"

if defined JAVA_HOME (
    if exist "%JAVA_HOME%\bin\java.exe" set "JAVA_CMD=%JAVA_HOME%\bin\java.exe"
    if exist "%JAVA_HOME%\bin\javac.exe" set "JAVAC_CMD=%JAVA_HOME%\bin\javac.exe"
)

where "%JAVA_CMD%" >nul 2>nul
if errorlevel 1 (
    echo [ERROR] Missing command: java
    echo [HINT] Install JDK 17 or newer, then set JAVA_HOME or add it to PATH.
    pause
    exit /b 1
)

where "%JAVAC_CMD%" >nul 2>nul
if errorlevel 1 (
    echo [ERROR] Missing command: javac
    echo [HINT] Install JDK 17 or newer, then set JAVA_HOME or add it to PATH.
    pause
    exit /b 1
)



if not exist build\classes mkdir build\classes
if not exist uart_temp mkdir uart_temp

(for /R src %%F in (*.java) do echo "%%F") > build\sources.txt

"%JAVAC_CMD%" --release 17 -encoding UTF-8 ^
    -cp "lib\jSerialComm-2.10.4.jar" ^
    -d build\classes ^
    @build\sources.txt
if errorlevel 1 (
    echo [ERROR] Java compile failed.
    pause
    exit /b 1
)

"%JAVA_CMD%" -Djava.io.tmpdir=uart_temp ^
    --enable-native-access=ALL-UNNAMED ^
    -Djava.library.path=lib ^
    -cp "build\classes;lib\jSerialComm-2.10.4.jar" ^
    app.App

endlocal
