import os
import sys
import subprocess
import shutil

def compile():
    # Change working directory to script's directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    print("===================================================")
    print("Compiling C++ JNI library (kinematics_jni.dll)...")
    print("===================================================")

    java_home = os.environ.get("JAVA_HOME")
    if not java_home:
        java_bin = shutil.which("java")
        if java_bin:
            java_dir = os.path.dirname(os.path.dirname(java_bin))
            if os.path.exists(os.path.join(java_dir, "include")):
                java_home = java_dir
                print(f"Auto-detected JAVA_HOME from PATH: {java_home}")
        
    if not java_home:
        print("[ERROR] JAVA_HOME environment variable is not set!")
        print("Please set JAVA_HOME to your JDK directory.")
        sys.exit(1)

    print(f"Using JAVA_HOME: {java_home}")

    vs_path = "C:\\Program Files\\Microsoft Visual Studio\\2022\\Community\\VC\\Auxiliary\\Build\\vcvars64.bat"
    if not os.path.exists(vs_path):
        vs_path = "C:\\Program Files (x86)\\Microsoft Visual Studio\\2022\\Community\\VC\\Auxiliary\\Build\\vcvars64.bat"

    if os.path.exists(vs_path):
        print(f"Found Visual Studio MSVC compiler tool: {vs_path}")
        # Extract environment variables set by vcvars64.bat
        cmd = f'"{vs_path}" && set'
        proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        stdout, stderr = proc.communicate()
        
        # Case-insensitive environment dictionary
        env = {}
        # Load current environment first (handling case-insensitivity)
        for k, v in os.environ.items():
            env[k.upper()] = v

        # Overlay variables from vcvars64.bat
        for line in stdout.splitlines():
            if "=" in line:
                key, val = line.split("=", 1)
                env[key.upper()] = val
        
        # Rebuild standard dictionary for subprocess
        sub_env = {k: v for k, v in env.items()}
        
        include_dirs = [
            os.path.join(java_home, "include"),
            os.path.join(java_home, "include", "win32")
        ]
        
        cl_cmd = [
            "cl.exe",
            "/LD",
            "/O2",
            "/EHsc",
            "/I", include_dirs[0],
            "/I", include_dirs[1],
            "kinematics_JniKinematics.cpp",
            "/Fe:..\\lib\\kinematics_jni.dll"
        ]
        
        print("Running: " + " ".join(f'"{x}"' if ' ' in x else x for x in cl_cmd))
        # Run without shell=True to avoid cmd.exe cache issues, using resolved PATH
        # We need to find cl.exe's full path since subprocess without shell may not search sub_env['PATH']
        cl_path = None
        for path_dir in env.get("PATH", "").split(os.pathsep):
            candidate = os.path.join(path_dir, "cl.exe")
            if os.path.isfile(candidate):
                cl_path = candidate
                break
                
        if cl_path:
            cl_cmd[0] = cl_path
            cl_proc = subprocess.run(cl_cmd, env=sub_env)
        else:
            # Fallback to shell invocation
            cl_proc = subprocess.run(" ".join(cl_cmd), env=sub_env, shell=True)
            
        if cl_proc.returncode == 0:
            print("[SUCCESS] Compiled successfully to ../lib/kinematics_jni.dll")
            # Clean up temporary files
            for f in ["kinematics_JniKinematics.obj", "..\\lib\\kinematics_jni.exp", "..\\lib\\kinematics_jni.lib"]:
                if os.path.exists(f):
                    os.remove(f)
        else:
            print("[ERROR] MSVC compilation failed.")
            sys.exit(cl_proc.returncode)
    else:
        # Fallback to g++
        print("Visual Studio MSVC compiler not found. Trying g++...")
        gxx = shutil.which("g++")
        if not gxx:
            print("[ERROR] Neither cl.exe (MSVC) nor g++ (MinGW) was found in PATH.")
            sys.exit(1)
            
        gxx_cmd = [
            "g++",
            "-shared",
            "-O3",
            "-fPIC",
            "-I", os.path.join(java_home, "include"),
            "-I", os.path.join(java_home, "include", "win32"),
            "kinematics_JniKinematics.cpp",
            "-o",
            "../lib/kinematics_jni.dll"
        ]
        print("Running: " + " ".join(f'"{x}"' if ' ' in x else x for x in gxx_cmd))
        gxx_proc = subprocess.run(gxx_cmd)
        if gxx_proc.returncode == 0:
            print("[SUCCESS] Compiled successfully to ../lib/kinematics_jni.dll")
        else:
            print("[ERROR] g++ compilation failed.")
            sys.exit(gxx_proc.returncode)

if __name__ == "__main__":
    compile()
