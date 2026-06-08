
    cd /workspace/prototypes/3d-cartographer-demo/

    # Launch Gazebo simulation
    gz sim room-v2.sdf &

    # Source ROS2
    source /opt/ros/jazzy/setup.bash

    ros2 launch slam_gazebo.launch.py &


Test:

    gz topic -t "/model/FLIP/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"
