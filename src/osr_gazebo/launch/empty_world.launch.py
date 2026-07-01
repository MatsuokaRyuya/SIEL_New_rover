import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. 新しいGazebo Sim (ros_gz_sim) の起動
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 2. ロボットモデル(URDF)の読み込み
    osr_urdf_path = get_package_share_directory('osr_gazebo')
    xacro_file = os.path.join(osr_urdf_path, 'urdf', 'osr.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    controller_spawn = Node(
        package='osr_gazebo',
        executable='osr_controller',
        output='screen'
    )
    
    # 3. 新しいGazebo Simへのエンティティ(機体)スポーン
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'rover',
                   '-z', '0.5'], # 地面に埋まらないようにZ軸を少し上げる
        output='screen'
    )

    # 4. コントローラのロード
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    rover_wheel_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'wheel_controller'],
        output='screen'
    )

    servo_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'servo_controller'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        controller_spawn,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    load_joint_state_controller,
                    rover_wheel_controller,
                    servo_controller,
                ],
            )
        )
    ])