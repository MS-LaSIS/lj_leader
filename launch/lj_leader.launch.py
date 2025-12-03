from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

DEBUG = os.getenv('DEBUG', default='False').lower() == 'true' or os.getenv('DEBUG', default='False').lower() == '1'

def generate_launch_description():
    # Declare launch arguments
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='Publish rate in Hz'
    )
    
    steering_topic_arg = DeclareLaunchArgument(
        'steering_topic',
        default_value='leader/steering_cmd',
        description='Topic name for steering ratio messages'
    )
    
    pedal_topic_arg = DeclareLaunchArgument(
        'pedal_topic',
        default_value='leader/pedal_cmd',
        description='Topic name for throttle/brake ratio messages'
    )
    
    log_level = 'debug' if DEBUG else 'info'
    
    # Create the lj_leader node
    lj_leader_node = Node(
        package='lj_leader',
        executable='lj_leader',
        name='lj_leader',
        output='screen',
        parameters=[{
            'publish_rate': LaunchConfiguration('publish_rate'),
            'steering_topic': LaunchConfiguration('steering_topic'),
            'pedal_topic': LaunchConfiguration('pedal_topic'),
            # Pin configurations for steering signals (AIN0-3)
            'steering_master1_pin': 'AIN1',
            'steering_slave1_pin': 'AIN0',
            'steering_master2_pin': 'AIN3',
            'steering_slave2_pin': 'AIN2',
            # Pin configurations for throttle/brake signals (AIN4-7)
            'throttle_master1_pin': 'AIN5',
            'throttle_slave1_pin': 'AIN4',
            'throttle_master2_pin': 'AIN7',
            'throttle_slave2_pin': 'AIN6',
            # Pin configurations for nominal voltages (AIN8-11)
            'nominal_vs_steer_master_pin': 'AIN11',
            'nominal_vs_steer_slave_pin': 'AIN11',
            'nominal_vs_throttle_master_pin': 'AIN8',
            'nominal_vs_throttle_slave_pin': 'AIN8',
            # Percentage limits for joystick ranges
            'steering_min_perc': 0.15,
            'steering_max_perc': 0.85,
            'throttle_min_perc': 0.23,
            'throttle_max_perc': 0.77,
        }],
        arguments=['--ros-args', '--log-level', log_level],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        publish_rate_arg,
        steering_topic_arg,
        pedal_topic_arg,
        lj_leader_node,
    ])
