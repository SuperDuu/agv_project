with open("search_result.txt", "w", encoding="utf-8") as out:
    with open(r"c:\Users\DELL\agv_project\arm\java_app\arm\src\kinematics\Kinematics.java", "r", encoding="utf-8") as f:
        for i, line in enumerate(f, 1):
            if "double L" in line or "double  L" in line or "static final double L" in line:
                out.write(f"Kinematics.java Line {i}: {line.strip()}\n")
