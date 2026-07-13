import os
import signal

for pid_str in os.listdir('/proc'):
    if pid_str.isdigit():
        try:
            with open(f'/proc/{pid_str}/cmdline', 'r') as f:
                cmd = f.read()
                if any(x in cmd for x in ['robot_state_publisher', 'joint_state_publisher', 'move_group', 'moveit_planner']):
                    print(f"Killing PID {pid_str}: {cmd}")
                    os.kill(int(pid_str), signal.SIGKILL)
        except Exception:
            pass
