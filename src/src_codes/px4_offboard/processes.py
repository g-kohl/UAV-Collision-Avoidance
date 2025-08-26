#!/usr/bin/env python3
import subprocess
import sys
import time


PX4_PATH = "~/PX4-Autopilot-ColAvoid" # path to PX4 repository
WORLD_NAME = "lawn"


def main(args=None):
    instance = int(sys.argv[1])
    spawn_position = sys.argv[2] if len(sys.argv) > 2 else "0.0,0.0"

    if instance == 0:
        subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", 
                        "MicroXRCEAgent udp4 -p 8888" + "; exec bash"]) # start the agent
    else:
        cd_command = f"cd {PX4_PATH} && "
        simulation_command = f"PX4_SYS_AUTOSTART=4001 PX4_GZ_WORLD={WORLD_NAME} PX4_GZ_MODEL_POSE={spawn_position} PX4_SIM_MODEL=gz_x500_lidar ./build/px4_sitl_default/bin/px4 -i {instance}"
                             # port                   world                     pose                               model

        time.sleep(3*instance)
        subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", 
                        cd_command + simulation_command + "; exec bash"]) # spawn the UAV

if __name__ == '__main__':
    main()