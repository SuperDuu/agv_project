import subprocess
import os

files_to_commit = {
    "arm_firmware_left/Core/Src/joint_control.c": "firmware: Adjust starting servo command angle on left arm",
    "arm_firmware_left/Core/Src/main.c": "firmware: Initialize and update encoders on left arm",
    "arm_firmware_left/Core/Src/servo.c": "firmware: Update initial servo angle for left arm joints",
    "arm_firmware_right/Core/Src/joint_control.c": "firmware: Adjust starting servo command angle on right arm",
    "arm_firmware_right/Core/Src/main.c": "firmware: Initialize and update encoders on right arm",
    "arm_firmware_right/Core/Src/servo.c": "firmware: Update initial servo angle for right arm joints",
    "cad/part/STL/ct1.STL": "cad: Update STL model for link ct1",
    "cad/part/V2/ct1.SLDPRT": "cad: Update SolidWorks part for link ct1",
    "cad/part/V2/vairb_v2.SLDASM": "cad: Update SolidWorks assembly file",
    "java_app/arm/src/gui/MainFrame.java": "gui: Optimize workspace slice calculation using joint-space FK",
    "java_app/arm/src/scratch/TestSlice.java": "scratch: Add test script for workspace slice calculation"
}

def run_cmd(cmd):
    print(f"Running: {' '.join(cmd)}")
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if res.returncode != 0:
        print(f"Error: {res.stderr}")
    else:
        print(f"Success: {res.stdout.strip()}")
    return res.returncode == 0

for file_path, commit_msg in files_to_commit.items():
    if os.path.exists(file_path):
        # Stage the file
        if run_cmd(["git", "add", file_path]):
            # Commit the file
            run_cmd(["git", "commit", "-m", commit_msg])
    else:
        print(f"File not found: {file_path}")

print("Done committing files!")
