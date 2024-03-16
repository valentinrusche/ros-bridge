import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='host', default_value='localhost'),
        launch.actions.DeclareLaunchArgument(name='port', default_value='2000'),
        launch.actions.DeclareLaunchArgument(name='timeout', default_value='10'),
        launch.actions.DeclareLaunchArgument(name='log_file', default_value='latest.log'),
        launch.actions.DeclareLaunchArgument(name='start', default_value='0'),
        launch.actions.DeclareLaunchArgument(name='duration', default_value='300'),
        launch.actions.DeclareLaunchArgument(name='camera_id', default_value='0'),
        launch.actions.DeclareLaunchArgument(name='replay_sensors', default_value='true'),
        launch.actions.DeclareLaunchArgument(name='keep_actors', default_value='false'),
        launch_ros.actions.Node(
            package='carla_replay_control',
            executable='carla_replay_control',
            name='carla_replay_control',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'host': launch.substitutions.LaunchConfiguration('host')},
                {'port': launch.substitutions.LaunchConfiguration('port')},
                {'timeout': launch.substitutions.LaunchConfiguration('timeout')},
                {'log_file': launch.substitutions.LaunchConfiguration('log_file')},
                {'start': launch.substitutions.LaunchConfiguration('start')},
                {'duration': launch.substitutions.LaunchConfiguration('duration')},
                {'camera_id': launch.substitutions.LaunchConfiguration('camera_id')},
                {'replay_sensors': launch.substitutions.LaunchConfiguration('replay_sensors')},
                {'keep_actors': launch.substitutions.LaunchConfiguration('keep_actors')},
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
