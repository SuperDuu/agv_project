# Run the Java Arm App

## Idea

The app is a plain Java Swing project. It needs:

- JDK 17 or newer.
- `lib/jSerialComm-2.10.4.jar`, already included.
- Python plus `pygame` only for the PS5 controller helper.
- Optional JNI library for fast IK: `kinematics_jni.dll` on Windows, `libkinematics_jni.so` on Ubuntu.

## Ubuntu

From this folder:

```bash
cd arm/java_app/arm
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r scripts/requirements.txt
./run.sh
```

If `python3 -m venv` is missing, install `python3-full` first.

If `./run.sh` says `java` or `javac` is missing, install JDK 17 or newer and make sure `JAVA_HOME` is set or both commands are in `PATH`.

To build the optional JNI solver on Ubuntu:

```bash
sudo apt install g++
export JAVA_HOME=/path/to/jdk-17
python3 cpp_jni/build_jni.py
./run.sh
```

The JNI build should create:

```text
lib/libkinematics_jni.so
```

When this file exists, the app enables the C++ JNI numerical solver automatically.

## Windows

Open Command Prompt in this folder:

```bat
cd arm\java_app\arm
python -m pip install -r scripts\requirements.txt
run.bat
```

If `run.bat` says `java` or `javac` is missing, install JDK 17 or newer, then set `JAVA_HOME` or add the JDK `bin` folder to `PATH`.

To build the optional JNI solver on Windows:

```bat
set JAVA_HOME=C:\Path\To\jdk-17
python cpp_jni\build_jni.py
run.bat
```

The JNI build should create:

```text
lib\kinematics_jni.dll
```

## Analysis

The run scripts compile every `.java` file under `src`, put `.class` files into `build/classes`, then run `app.App` with the correct classpath separator for each OS.

On Ubuntu the classpath separator is `:`.
On Windows the classpath separator is `;`.

## Effectiveness Proof

Let `S` be the set of Java source files under `src`. Both scripts compute the same `S`, compile it into `build/classes`, and run the same main class `app.App`.

The only OS-dependent values are:

- classpath separator: `:` vs `;`
- native JNI filename: `.dll` vs `.so`
- Python executable name: `python` vs `python3`

The scripts and code now choose those values per OS. Therefore the remaining requirement is only that the same external tools exist: JDK 17+, Python, and optional C++ compiler for JNI.
