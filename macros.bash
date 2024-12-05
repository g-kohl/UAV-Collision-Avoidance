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

function sim() {
    local uav_number=${1:-1}
    local spawn_configuration=${2:-l}

    ros2 launch px4_offboard offboard_velocity_control.launch.py \
        uav_number:=$uav_number \
        spawn_configuration:=$spawn_configuration
}

function kgz() {
    ps aux | grep gz | grep -v grep | awk '{print $2}' | xargs kill -9
}
