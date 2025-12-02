from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from launch.launch_description_sources import FrontendLaunchDescriptionSource  # Add this import


def generate_launch_description():
    """
    Launch the complete pipeline with sequential delays:
    1. foxglove_bridge (starts immediately)
    2. DLIO (after 5 seconds)
    3. Open3D SLAM (after 15 seconds total; 10 seconds after DLIO)
    4. Vehicle Simulator (after 20 seconds total; 5 seconds after Open3D)
    5. Far Planner (after 25 seconds total; 5 seconds after Vehicle)
    """
    
    # Find package paths
    vehicle_simulator_pkg = FindPackageShare('vehicle_simulator')
    far_planner_pkg = FindPackageShare('far_planner')
    foxglove_bridge_pkg = FindPackageShare('foxglove_bridge')
    
    # 1. Launch Foxglove Bridge immediately
    foxglove_bridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([
            foxglove_bridge_pkg, '/launch/foxglove_bridge_launch.xml'
        ])
    )

    # 2. Launch DLIO (after 5s)
    dlio_launch = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('direct_lidar_inertial_odometry'), '/launch/dlio.launch.py'
                ]),
                launch_arguments={
                    'rviz': 'true',
                    'pointcloud_topic': '/livox/lidar',
                    'imu_topic': '/livox/imu',
                }.items()
            )
        ]
    )

    # 3. Launch Open3D SLAM after 15 seconds total
    open3d_slam_launch = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('open3d_slam_ros'), '/launch/open3d_launch.py'
                ]),
                launch_arguments={
                    'use_sim_time': 'true',
                    'launch_rviz': 'false',
                    'cloud_topic': '/dlio_registered_scan',
                }.items()
            )
        ]
    )

    # 4. Launch Vehicle Simulator after 20 seconds total
    vehicle_simulator_launch = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    vehicle_simulator_pkg, '/launch/system_real_robot.launch'
                ])
            )
        ]
    )
    
    # 5. Launch Far Planner after 25 seconds total
    far_planner_launch = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    far_planner_pkg, '/launch/far_planner.launch'
                ])
            )
        ]
    )

    # Foxglove Bridge already defined above (starts immediately)
    
    return LaunchDescription([
        foxglove_bridge_launch,  # 0s - Foxglove Bridge
        dlio_launch,             # 5s - DLIO
        open3d_slam_launch,      # 15s - Open3D SLAM
        vehicle_simulator_launch,# 20s - Vehicle Simulator
        far_planner_launch,      # 25s - Far Planner
    ])
