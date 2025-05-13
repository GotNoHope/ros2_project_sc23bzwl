from setuptools import setup

package_name = 'ros2_project_sc23bzwl'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
         ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@domain.com',
    description='COMP3631 ROS2 Project Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'first_step = ros2_project_sc23bzwl.first_step:main',
            'second_step = ros2_project_sc23bzwl.second_step:main',
            'third_step = ros2_project_sc23bzwl.third_step:main',
            'fourth_step = ros2_project_sc23bzwl.fourth_step:main',
            'robot_controller = ros2_project_sc23bzwl.robot_controller:main',
        ],
    },
)
