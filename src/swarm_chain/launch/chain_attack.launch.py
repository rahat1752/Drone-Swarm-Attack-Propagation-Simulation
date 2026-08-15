from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("swarm_chain")

    mission_file = LaunchConfiguration("mission_file")
    formation_file = LaunchConfiguration("formation_file")
    attack_config = LaunchConfiguration("attack_config")
    output_dir = LaunchConfiguration("output_dir")
    follower_speed_mps = LaunchConfiguration("follower_speed_mps")
    mission_start_mode = LaunchConfiguration("mission_start_mode")
    fallback_start_after_s = LaunchConfiguration("fallback_start_after_s")

    default_mission = PathJoinSubstitution([
        pkg_share,
        "configs",
        "sweeping_mission.json"
    ])

    default_formation = PathJoinSubstitution([
        pkg_share,
        "configs",
        "formation_inverted_v_tree.json"
    ])

    default_attack_config = PathJoinSubstitution([
        pkg_share,
        "configs",
        "attack_config.json"
    ])

    return LaunchDescription([

        DeclareLaunchArgument(
            "mission_file",
            default_value=default_mission,
            description="Path to mission JSON file."
        ),

        DeclareLaunchArgument(
            "formation_file",
            default_value=default_formation,
            description="Path to formation JSON file."
        ),

        DeclareLaunchArgument(
            "attack_config",
            default_value=default_attack_config,
            description="Path to attack configuration JSON file."
        ),

        DeclareLaunchArgument(
            "output_dir",
            default_value="/tmp/swarm_chain_attack",
            description="Directory where CSV logs will be saved."
        ),

        DeclareLaunchArgument(
            "follower_speed_mps",
            default_value="2.0",
            description="Maximum follower virtual target speed."
        ),

        DeclareLaunchArgument(
            "mission_start_mode",
            default_value="leader_armed",
            description="Logging start mode. Use leader_armed or immediate."
        ),

        DeclareLaunchArgument(
            "fallback_start_after_s",
            default_value="-1.0",
            description="Fallback logger start time. Negative disables fallback."
        ),

        #######################################################################
        # 1. Raw state broadcaster
        #
        # This should publish:
        #   /swarm/drone_i_state_raw
        #
        # attack_manager will convert raw state into final state:
        #   /swarm/drone_i_state
        #######################################################################

        Node(
            package="swarm_chain",
            executable="state_broadcaster",
            name="state_broadcaster",
            output="screen"
        ),

        #######################################################################
        # 2. Attack manager
        #
        # Input:
        #   /swarm/drone_i_state_raw
        #
        # Output:
        #   /swarm/drone_i_state
        #
        # It applies spoofing, jamming, delay, jitter, or replay depending on
        # attack_config.json.
        #######################################################################

        Node(
            package="swarm_chain",
            executable="attack_manager",
            name="attack_manager",
            output="screen",
            parameters=[{
                "attack_config": attack_config,
            }]
        ),

        #######################################################################
        # 3. Chain / inverted-V formation controller
        #######################################################################

        Node(
            package="swarm_chain",
            executable="chain_controller",
            name="chain_controller",
            output="screen",
            parameters=[{
                "formation_file": formation_file,
                "mission_file": mission_file,
                "follower_speed_mps": follower_speed_mps,
            }]
        ),

        #######################################################################
        # 4. State logger
        #######################################################################

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
            }]
        ),

        #######################################################################
        # 5. Leader mission controller
        #######################################################################

        Node(
            package="swarm_chain",
            executable="mission_controller",
            name="mission_controller",
            output="screen",
            parameters=[{
                "mission_file": mission_file,
                "tracking_error_limit_m": 3.0,
                "acceptance_radius_m": 0.5,
            }]
        ),
    ])
