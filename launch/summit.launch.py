from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # --- Declare and configure launch arguments ---
    declare_slam_arg = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Enable SLAM. If True, SLAM is used. If False, a pre-existing map is used.",
    )

    world_file_path = PathJoinSubstitution(
        [FindPackageShare("rosa_summit"), "world", "small_house.world"]
    )
    map_file_path = PathJoinSubstitution(
        [FindPackageShare("rosa_summit"), "maps", "default.yaml"]
    )
    summit_xl_simulation_launch_file = PathJoinSubstitution(
        [FindPackageShare("icclab_summit_xl"), "launch", "summit_xl_simulation_ign.launch.py"]
    )
    summit_xl_nav2_launch_file = PathJoinSubstitution(
        [FindPackageShare("icclab_summit_xl"), "launch", "summit_xl_nav2.launch.py"]
    )
    explore_lite_launch_file = PathJoinSubstitution(
        [FindPackageShare("explore_lite"), "launch", "explore.launch.py"]
    )

    # Static TFs for laser sensor frames.
    # Ignition Gazebo fuses fixed joints and names sensor frames as:
    #   <model>/<fused_link>/<sensor_name>
    # Since base_footprint->base_link->front_laser_base_link->front_laser_link are
    # all fixed joints, Ignition fuses them all into base_footprint.
    #
    # Calculated from URDF:
    #   base_footprint -> base_link:            z=0.127
    #   base_link -> front_laser_base_link:     (0.356, -0.251, 0.1566), yaw=-pi/4
    #   front_laser_base_link -> front_laser_link: z=0.055
    #   Total:                                  (0.356, -0.251, 0.3386), yaw=-0.7854
    #
    #   base_link -> rear_laser_base_link:      (-0.35, 0.251, 0.1566), yaw=3*pi/4
    #   Total:                                  (-0.35, 0.251, 0.3386), yaw=2.3562

    _front_args = [
        '--x', '0.356', '--y', '-0.251', '--z', '0.3386',
        '--yaw', '-0.7854', '--pitch', '0.0', '--roll', '0.0',
        '--frame-id', 'base_footprint',
        '--child-frame-id', 'summit/base_footprint/front_laser_sensor',
    ]
    _rear_args = [
        '--x', '-0.35', '--y', '0.251', '--z', '0.3386',
        '--yaw', '2.3562', '--pitch', '0.0', '--roll', '0.0',
        '--frame-id', 'base_footprint',
        '--child-frame-id', 'summit/base_footprint/rear_laser_sensor',
    ]

    # Global namespace: makes laser visible in RViz (reads /tf_static)
    static_tf_front_global = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=_front_args, output='screen',
    )
    static_tf_rear_global = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=_rear_args, output='screen',
    )
    # /summit namespace: makes laser visible to SLAM/Nav2 (reads /summit/tf_static)
    static_tf_front_summit = Node(
        package='tf2_ros', executable='static_transform_publisher',
        namespace='summit',
        remappings=[('/tf_static', 'tf_static')],
        arguments=_front_args, output='screen',
    )
    static_tf_rear_summit = Node(
        package='tf2_ros', executable='static_transform_publisher',
        namespace='summit',
        remappings=[('/tf_static', 'tf_static')],
        arguments=_rear_args, output='screen',
    )

    # Arm controller spawner (gz_ros2_control handles arm joints only)
    spawner_arm = TimerAction(
        period=25.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller", "--controller-manager", "/summit/controller_manager"],
            output="screen",
        )],
    )

    # Send initial arm trajectory to hold it in a folded position
    hold_arm = TimerAction(
        period=35.0,
        actions=[ExecuteProcess(
            cmd=[
                "ros2", "action", "send_goal",
                "/summit/arm_controller/follow_joint_trajectory",
                "control_msgs/action/FollowJointTrajectory",
                "{trajectory: {joint_names: [arm_shoulder_pan_joint, arm_shoulder_lift_joint, arm_elbow_joint, arm_wrist_1_joint, arm_wrist_2_joint, arm_wrist_3_joint], points: [{positions: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0], velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}}",
            ],
            output="screen",
        )],
    )

    # Bridge Ignition topics to ROS2
    gz_bridge = TimerAction(
        period=20.0,
        actions=[Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/model/summit/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                "/model/summit/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
                "/model/summit/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model",
                "/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
                "/tf_static@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
                "/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock",
                "/summit/merged_laser_scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            ],
            output="screen",
        )],
    )

    # Safety velocity filter: /summit/cmd_vel -> /model/summit/cmd_vel (DIRECT)
    # Monitors laser, scales/stops velocity when obstacles are close.
    # Replaces the old relay — this IS the relay + safety filter combined.
    safety_filter = Node(
        package="rosa_summit",
        executable="safety_filter",
        output="screen",
    )

    # Relay /model/summit/odom -> /summit/odom (Nav2 expects this topic)
    relay_odom = TimerAction(
        period=21.0,
        actions=[Node(
            package="topic_tools",
            executable="relay",
            arguments=["/model/summit/odom", "/summit/odom"],
            output="screen",
        )],
    )

    # Relay /tf -> /summit/tf so Nav2/SLAM see odom->base_footprint
    relay_tf = TimerAction(
        period=21.0,
        actions=[Node(
            package="topic_tools",
            executable="relay",
            arguments=["/tf", "/summit/tf"],
            output="screen",
        )],
    )

    # Relay /map -> /summit/map: slam_toolbox hardcodes "/map" (absolute) and
    # ignores the /summit namespace, so Nav2 components subscribed to
    # /summit/map never receive the map without this relay.
    relay_map = TimerAction(
        period=22.0,
        actions=[Node(
            package="topic_tools",
            executable="relay",
            arguments=["/map", "/summit/map"],
            output="screen",
        )],
    )

    # Republish /summit/merged_laser_scan → /scan with frame_id rewritten to
    # 'base_footprint'.  slam_toolbox hardcodes scan_topic=/scan and needs
    # the scan frame to be resolvable in its TF tree.  Ignition names sensor
    # frames as 'summit/base_footprint/front_laser_sensor' which is hard to
    # wire up reliably; using base_footprint directly removes that dependency.
    scan_republisher = Node(
        package="rosa_summit",
        executable="scan_republisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    actions_if_slam = [
        static_tf_front_global,
        static_tf_rear_global,
        static_tf_front_summit,
        static_tf_rear_summit,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(summit_xl_simulation_launch_file),
            launch_arguments={"world": world_file_path}.items(),
        ),
        gz_bridge,
        safety_filter,
        relay_odom,
        relay_tf,
        relay_map,
        scan_republisher,
        spawner_arm,
        hold_arm,
        # Delay Nav2 start until bridge (t=20s) and relays (t=21s) are up
        # so that TF frames (odom, map) are available when costmaps initialize
        TimerAction(
            period=25.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(summit_xl_nav2_launch_file),
                launch_arguments={"slam": "True", "rviz": "True"}.items(),
            )],
        ),
        TimerAction(
            period=30.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(explore_lite_launch_file),
                launch_arguments={
                    "namespace": "/summit",
                    "use_sim_time": "True",
                }.items(),
            )],
        ),
        TimerAction(
            period=32.0,
            actions=[ExecuteProcess(
                cmd=[
                    "ros2", "topic", "pub", "--once",
                    "/summit/explore/resume", "std_msgs/msg/Bool", "{data: false}",
                ],
            )],
        ),
    ]

    actions_if_no_slam = [
        static_tf_front_global,
        static_tf_rear_global,
        static_tf_front_summit,
        static_tf_rear_summit,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(summit_xl_simulation_launch_file),
            launch_arguments={"world": world_file_path}.items(),
        ),
        gz_bridge,
        safety_filter,
        relay_odom,
        relay_tf,
        scan_republisher,
        spawner_arm,
        hold_arm,
        # Delay Nav2 start until bridge and relays are up
        TimerAction(
            period=25.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(summit_xl_nav2_launch_file),
                launch_arguments={"map": map_file_path, "rviz": "True"}.items(),
            )],
        ),
    ]

    def evaluate_slam_and_select_actions(context, *args, **kwargs):
        slam_value = context.launch_configurations["slam"]
        if slam_value.lower() == "true":
            return actions_if_slam
        else:
            return actions_if_no_slam

    return LaunchDescription(
        [declare_slam_arg, OpaqueFunction(function=evaluate_slam_and_select_actions)]
    )
