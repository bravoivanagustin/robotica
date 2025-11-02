from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Use simulation time for all nodes
        Node(
            package='modelo_diferencial',
            executable='pioneer_odometry_node',
            name='pioneer_odometry',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='lazo_abierto',
            executable='trajectory_follower',
            name='trajectory_follower',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='lazo_abierto',
            executable='trajectory_generator',
            name='trajectory_generator',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'stepping': 0.1},
                {'trajectory_type': 'sin'},
                {'total_time': 15.0},
                {'amplitude': 1.0},
                {'cycles': 1.0},
                {'spline_waypoints': [
                    0., 0., 0., 0.,
                    100., 5., 0., 1.57,
                    200., 5., 5., 3.14,
                    300., 0., 5., 4.71,
                    400., 0., 0., 0.
                ]}
            ]
        )
    ])
# Note: each waypoint must have 4 values: time(sec), position_x(m), position_y(m), orientation(rad)