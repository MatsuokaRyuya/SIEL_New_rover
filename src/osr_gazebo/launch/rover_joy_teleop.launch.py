from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # joy_node の起動
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/event0'
            }]
        ),
        # teleop_node の起動
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[{
                'axis_linear.x': 1,
                'axis_angular.yaw': 0,
                'scale_linear.x': 0.5,
                'scale_angular.yaw': 0.5
            }]
        )
        # Node(
        #     package='teleop_twist_joy',
        #     executable='teleop_node',
        #     name='teleop_node',
        #     # 固まったときに即座に終了させる設定
        #     sigterm_timeout='5',
        #     sigkill_timeout='5',
        #     parameters=[...]
        # )

    ])

##How to use USB controller in WSL2
## launch power shell as administrator and run the following command to enable USB device support in WSL2:
# usbipd list
# usbipd attach --wsl --busid 1-8
# sudo chmod 666 /dev/input/event0 の後にこのlaunchファイル