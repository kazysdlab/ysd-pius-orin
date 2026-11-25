import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Load config.yaml for node parameters
    config_path = '/home/orin/dev/ws/colcon_ws/config.yaml'
    camera_params = {}
    dummy_joy_params = {}
    parser_params = {}
    mux_params = {}
    save_params = {}
    sender_params = {}
    default_model_path = '/home/orin/dev/ws/colcon_ws/models/model.pth'

    if os.path.exists(config_path):
        try:
            import yaml
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f)
            camera_params = cfg.get('camera', {})
            dummy_joy_params = cfg.get('dummy_joy', {})
            parser_params = cfg.get('parser', {})
            mux_params = cfg.get('mux', {})
            save_params = cfg.get('save', {})
            sender_params = cfg.get('sender', {})
            default_model_path = cfg.get('inference', {}).get('model_path', default_model_path)
        except Exception as e:
            print(f"Error loading config.yaml in launch description: {e}")

    # Parameters to toggle dummy simulator vs physical hardware independently
    use_dummy_camera_arg = DeclareLaunchArgument(
        'use_dummy_camera',
        default_value='false',
        description='If true, runs dummy camera node. If false, runs physical csi_camera node.'
    )
    
    use_dummy_joy_arg = DeclareLaunchArgument(
        'use_dummy_joy',
        default_value='false',
        description='If true, runs dummy joy node. If false, runs physical dualsense node.'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model_path,
        description='Path to the trained CNN model file (.pth). Defaults to inference.model_path in config.yaml'
    )

    dry_run_arg = DeclareLaunchArgument(
        'dry_run',
        default_value='false',
        description='If true, the sender never writes to the serial port (safe for desk testing).'
    )

    use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='If true, runs the WitMotion IMU/GPS node (optional sensor; runs fine when unplugged).'
    )

    use_wheel_arg = DeclareLaunchArgument(
        'use_wheel',
        default_value='true',
        description='If true, runs the wheel speed sensor node (optional sensor on the 40-pin header GPIO).'
    )

    use_dummy_camera = LaunchConfiguration('use_dummy_camera')
    use_dummy_joy = LaunchConfiguration('use_dummy_joy')
    model_path = LaunchConfiguration('model_path')
    dry_run = LaunchConfiguration('dry_run')
    use_imu = LaunchConfiguration('use_imu')
    use_wheel = LaunchConfiguration('use_wheel')

    return LaunchDescription([
        use_dummy_camera_arg,
        use_dummy_joy_arg,
        model_path_arg,
        dry_run_arg,
        use_imu_arg,
        use_wheel_arg,

        # --- Camera Nodes ---
        Node(
            package='pius_control',
            executable='dummy_camera_node',
            name='dummy_camera_node',
            output='screen',
            condition=IfCondition(use_dummy_camera),
            parameters=[camera_params],
            emulate_tty=True
        ),
        Node(
            package='camera',
            executable='csi_camera_node',
            name='csi_camera_node',
            output='screen',
            condition=UnlessCondition(use_dummy_camera),
            parameters=[camera_params],
            respawn=True,
            respawn_delay=2.0,
            emulate_tty=True
        ),

        # --- Joystick/Controller Nodes ---
        Node(
            package='pius_control',
            executable='dummy_joy_node',
            name='dummy_joy_node',
            output='screen',
            condition=IfCondition(use_dummy_joy),
            parameters=[dummy_joy_params],
            emulate_tty=True
        ),
        Node(
            package='gamepad',
            executable='dualsense_node',
            name='dualsense_node',
            output='screen',
            condition=UnlessCondition(use_dummy_joy),
            parameters=[dummy_joy_params],
            respawn=True,
            respawn_delay=2.0,
            emulate_tty=True
        ),

        # --- Common Core Autopilot Nodes ---
        Node(
            package='pius_control',
            executable='parser_node',
            name='parser_node',
            output='screen',
            parameters=[parser_params],
            emulate_tty=True
        ),
        Node(
            package='pius_control',
            executable='mux_node',
            name='mux_node',
            output='screen',
            parameters=[mux_params],
            emulate_tty=True
        ),
        Node(
            package='pius_control',
            executable='save_node',
            name='save_node',
            output='screen',
            parameters=[save_params],
            emulate_tty=True
        ),
        Node(
            package='pius_control',
            executable='inference_node',
            name='inference_node',
            output='screen',
            parameters=[{'model_path': model_path}],
            emulate_tty=True
        ),
        Node(
            package='pius_control',
            executable='waypoint_controller_node',
            name='waypoint_controller_node',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='pius_control',
            executable='sender_node',
            name='sender_node',
            output='screen',
            parameters=[sender_params, {'dry_run': ParameterValue(dry_run, value_type=bool)}],
            emulate_tty=True
        ),
        Node(
            package='pius_control',
            executable='health_monitor_node',
            name='health_monitor_node',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='pius_control',
            executable='imu_node',
            name='imu_node',
            output='screen',
            condition=IfCondition(use_imu),
            respawn=True,
            respawn_delay=2.0,
            emulate_tty=True
        ),
        Node(
            package='pius_control',
            executable='wheel_speed_node',
            name='wheel_speed_node',
            output='screen',
            condition=IfCondition(use_wheel),
            respawn=True,
            respawn_delay=2.0,
            emulate_tty=True
        ),
        Node(
            package='pius_control',
            executable='dashboard_node',
            name='dashboard_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            emulate_tty=True
        ),
    ])
