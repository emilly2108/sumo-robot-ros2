"""Robot launch; the color manager owns and recovers the color process."""
from datetime import datetime
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def report_node_exit(node, label):
    """A process exiting never shuts down the rest of this launch."""
    def on_exit(event, context):
        if context.is_shutdown:
            return []
        return [LogInfo(
            msg=f"노드 종료: {label}, 종료코드={event.returncode}; "
                "다른 노드와 launch는 계속 실행합니다."
        )]
    return RegisterEventHandler(OnProcessExit(target_action=node, on_exit=on_exit))


def configure_nodes(context):
    configured_dir = LaunchConfiguration('color_recovery_log_dir').perform(context)
    record = (Path(configured_dir).expanduser().resolve() if configured_dir else
              Path.home() / '.ros' / 'color_recovery' /
              (datetime.now().strftime('%Y%m%d_%H%M%S_%f') + f'_{os.getpid()}'))
    record.mkdir(parents=True, exist_ok=True)
    profile = Path(get_package_share_directory('sumo_robot')) / 'config' / 'color_recovery_fastdds.xml'
    if not profile.is_file():
        raise RuntimeError(f'컬러 복구 통신 설정이 없습니다. 복사본 패키지를 빌드하세요: {profile}')
    cmd_topic = LaunchConfiguration('cmd_vel_topic')
    motor = Node(
        package='sumo_robot', executable='motor_node', name='motor_node',
        output='screen', emulate_tty=True, respawn=False,
        condition=IfCondition(LaunchConfiguration('enable_motor')),
        remappings=[('/cmd_vel', cmd_topic)],
    )
    vision = Node(
        package='sumo_robot', executable='vision_node', name='vision_node',
        output='screen', emulate_tty=True, respawn=False,
    )
    # This is a non-ROS parent process. It starts the real color_node with the
    # usual name/topic and replaces only that child on failure.
    color_command = [sys.executable, '-m', 'sumo_robot.color_recovery_supervisor',
                     '--record-dir', str(record), '--node-name', 'color_node',
                     '--topic', '/color_sensor', '--hang-seconds', '2.0',
                     '--stale-reset-seconds', '3.0']
    if LaunchConfiguration('enable_color_bus_recovery').perform(context).lower() in ('true','1'):
        color_command += ['--enable-bus-recovery','--bus-recovery-socket',
                          LaunchConfiguration('color_bus_recovery_socket').perform(context)]
    child_runner = LaunchConfiguration('color_child_runner').perform(context)
    if child_runner:
        color_command += ['--child-runner', child_runner]
    color = ExecuteProcess(
        cmd=color_command,
        name='color_recovery_manager', output='screen', emulate_tty=True,
        respawn=False,
    )
    brain = Node(
        package='sumo_robot', executable='hfsm_brain_node', name='hfsm_brain_node',
        output='screen', emulate_tty=True, respawn=False,
        remappings=[('/cmd_vel', cmd_topic)],
    )
    switch = Node(
        package='sumo_robot', executable='switch_node', name='switch_node',
        output='screen', emulate_tty=True, respawn=False,
        remappings=[('/cmd_vel', cmd_topic)],
    )
    return [
        # Apply to all participants in this launch, so a restarted color
        # participant can reconnect promptly to the existing brain.
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        SetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE', str(profile)),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', str(profile)),
        LogInfo(msg=f'컬러 복구 관리자: 진행 정지 2초 / 정상 데이터 부재 3초, 기록={record}'),
        report_node_exit(motor, 'motor_node'),
        report_node_exit(vision, 'vision_node'),
        report_node_exit(color, '컬러 복구 관리자'),
        report_node_exit(brain, 'hfsm_brain_node'),
        report_node_exit(switch, 'switch_node'),
        motor, vision, color, brain,
        TimerAction(period=LaunchConfiguration('switch_start_delay_sec'), actions=[switch],
                    condition=IfCondition(LaunchConfiguration('enable_switch'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('switch_start_delay_sec', default_value='5.0',
                              description='Delay physical switch synchronization during startup'),
        DeclareLaunchArgument('enable_motor', default_value='true',
                              description='Start motor hardware node'),
        DeclareLaunchArgument('enable_switch', default_value='true',
                              description='Start physical READY/FIGHT switch node'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel',
                              description='Velocity command topic shared by brain and motor'),
        DeclareLaunchArgument('color_recovery_log_dir', default_value='',
                              description='Color recovery records; empty creates a unique ~/.ros/color_recovery directory'),
        DeclareLaunchArgument('enable_color_bus_recovery', default_value='true',
                              description='Use the prepared privileged I2C recovery helper'),
        DeclareLaunchArgument('color_bus_recovery_socket', default_value='/run/sumo-color-i2c/recovery.sock',
                              description='Restricted local I2C recovery helper socket'),
        DeclareLaunchArgument('color_child_runner', default_value='',
                              description='Diagnostic child script; empty starts the real color_node'),
        OpaqueFunction(function=configure_nodes),
    ])
