from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            namespace='urs',
            parameters=[
                {"camera_name": "oak"},
                {"unite_imu_method": 2},
                {"enable_gyro": True},
                {"enable_accel": True},
                {"gyro_qos": "SENSOR_DATA"},
                {"accel_qos": "SENSOR_DATA"},
                {"accel_info_qos": "SENSOR_DATA"},
                {"gyro_info_qos": "SENSOR_DATA"},
                {"publish_tf": True},
                {"enable_depth": True},
                {"enable_color": True},
                {"enable_rgbd": True},
                {"enable_sync": True},
                {"align_depth.enable": True},
                {"clip_distance": 2.0},
                {"depth_module.profile": "640x480x30"},
                {"rgb_camera.profile": "640x480x30"},
                {"pointcloud.enable": True},
                {"pointcloud.texture_stream": "depth"},  # Change to "color" if RGB is working
            ],
            output='screen'
        )
    ])
