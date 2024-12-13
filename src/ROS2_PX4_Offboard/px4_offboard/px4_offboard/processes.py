#!/usr/bin/env python3
import subprocess
import sys
import time


def main(args=None):
    instance = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    spawn_position = sys.argv[2] if len(sys.argv) > 2 else f"0,{instance-1}"

    if instance == 0:
        subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", 
                        "MicroXRCEAgent udp4 -p 8888" + "; exec bash"]) # start the agent
    else:
        cd_command = "cd ~/PX4-Autopilot-ColAvoid && "
        simulation_command = f"PX4_SYS_AUTOSTART=4001 PX4_GZ_WORLD=default PX4_GZ_MODEL_POSE={spawn_position} PX4_SIM_MODEL=gz_x500_lidar ./build/px4_sitl_default/bin/px4 -i {instance}"

        time.sleep(5*instance)
        subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", 
                        cd_command + simulation_command + "; exec bash"]) # spawn the UAV

if __name__ == '__main__':
    main()