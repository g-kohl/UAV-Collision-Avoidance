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
    OPTIND=1

    local uav_number=1
    local spawn_configuration="l"
    local mission_mode="false"

    while getopts "n:c:m" opt; do
        case $opt in
            n) uav_number="$OPTARG" ;;
            c) spawn_configuration="$OPTARG" ;;
            m) mission_mode="true" ;;
        esac
    done

    ros2 launch px4_offboard offboard_velocity_control.launch.py \
        uav_number:=$uav_number \
        spawn_configuration:=$spawn_configuration \
        mission_mode:=$mission_mode
}


function loadmission() {
    cp "src/mission.txt" "install/px4_offboard/share/px4_offboard/mission.txt"
}

function kgz() {
    ps aux | grep gz | grep -v grep | awk '{print $2}' | xargs kill -9
}
