from setuptools import setup
import os
from glob import glob

package_name = "ros_gz_example_application"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="your_name",
    maintainer_email="your_email@example.com",
    description="Application-specific implementations for ROS-GZ integration",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "go_to_goal = ros_gz_example_application.go_to_goal:main",
            "ball_spawner = ros_gz_example_application.ball_spawner:main",
            "path_publisher = ros_gz_example_application.path_publisher:main",
            "odom_tf_broadcaster = ros_gz_example_application.odom_tf_broadcaster:main",
            'filter_odom = odometry_filter.filter_odom:main',
        ],
    },
    data_files=[
        ("share/" + package_name, ["package.xml"]),
    ],
)
