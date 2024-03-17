from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='challenge2',
            executable='signal_generator'
		),
        Node(
            package='challenge2',
            executable='signal_reconstruction'
		),
        Node(
            package = 'rqt_plot',
            executable = 'rqt_plot',
            parameters = [{'args': '/signal/wave/data /signal_reconstructed/wave'}] 
        )
	])