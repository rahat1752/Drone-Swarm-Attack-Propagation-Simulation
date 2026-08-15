from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("swarm_chain")

    mission_file = LaunchConfiguration("mission_file")
    formation_file = LaunchConfiguration("formation_file")
    output_dir = LaunchConfiguration("output_dir")
    attack_config = LaunchConfiguration("attack_config")
    follower_speed_mps = LaunchConfiguration("follower_speed_mps")
    mission_start_mode = LaunchConfiguration("mission_start_mode")
    fallback_start_after_s = LaunchConfiguration("fallback_start_after_s")

    default_mission = PathJoinSubstitution([pkg_share, "configs", "sweeping_mission.json"])
    default_formation = PathJoinSubstitution([pkg_share, "configs", "formation_inverted_v_tree.json"])
    default_attack_config = PathJoinSubstitution([pkg_share, "configs", "baseline_config.json"])

    return LaunchDescription([
        DeclareLaunchArgument("mission_file", default_value=default_mission),
        DeclareLaunchArgument("formation_file", default_value=default_formation),
        DeclareLaunchArgument("output_dir", default_value="/tmp/swarm_chain_baseline"),
        DeclareLaunchArgument("attack_config", default_value=default_attack_config),
        DeclareLaunchArgument("follower_speed_mps", default_value="2.0"),
        DeclareLaunchArgument("mission_start_mode", default_value="leader_armed"),
        DeclareLaunchArgument("fallback_start_after_s", default_value="-1.0"),

        Node(package="swarm_chain", executable="state_broadcaster", name="state_broadcaster", output="screen"),

        # Benign pass-through attack manager.  The formation controller subscribes
        # to /swarm/drone_i_state, while state_broadcaster publishes the raw
        # /swarm/drone_i_state_raw topics.  baseline_config.json disables all
        # manipulation but preserves the same data path as attack runs.
        Node(
            package="swarm_chain",
            executable="attack_manager",
            name="attack_manager",
            output="screen",
            parameters=[{"attack_config": attack_config}],
        ),

        Node(
            package="swarm_chain",
            executable="chain_controller",
            name="chain_controller",
            output="screen",
            parameters=[{
                "formation_file": formation_file,
                "mission_file": mission_file,
                "follower_speed_mps": follower_speed_mps,
            }],
        ),

        Node(
            package="swarm_chain",
            executable="state_logger",
            name="state_logger",
            output="screen",
            parameters=[{
                "formation_file": formation_file,
                "output_dir": output_dir,
                "sample_period_s": 0.1,
                "mission_start_mode": mission_start_mode,
                "fallback_start_after_s": fallback_start_after_s,
            }],
        ),

        Node(
            package="swarm_chain",
            executable="mission_controller",
            name="mission_controller",
            output="screen",
            parameters=[{
                "mission_file": mission_file,
                "tracking_error_limit_m": 3.0,
                "acceptance_radius_m": 0.5,
            }],
        ),
    ])
