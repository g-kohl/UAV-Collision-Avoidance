function setros() {
    source /opt/ros/humble/setup.bash
}

function setup() {
    source install/setup.bash
}

function buildall() {
    colcon build && setup
}

function build() {
    colcon build --packages-select px4_offboard && setup
}

function remodel() {
    cp -r models/* ~/PX4-Autopilot-ColAvoid/Tools/simulation/gz/models/
}

function loadmission() {
    cp "mission.txt" "install/px4_offboard/share/px4_offboard/mission.txt"
}

function sim() {
    local uav_number=${1:-1}
    local spawn_configuration=${2:-l}
    local mission_mode=${3:-f}

    ros2 launch px4_offboard offboard_velocity_control.launch.py \
        uav_number:=$uav_number \
        spawn_configuration:=$spawn_configuration \
        mission_mode:=$mission_mode
}

function kgz() {
    ps aux | grep gz | grep -v grep | awk '{print $2}' | xargs kill -9
}
