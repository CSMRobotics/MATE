from setuptools import setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hermanoid',
    maintainer_email='lucas@birdseyerobotics.com',
    description='The core decisionmaker for the CSM Robotics Mate ROV on the robot (ROV) side. Takes in inputs from all throughout the system and makes high-level decisions',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'RobotControl = robot_control.RobotControl:main'
        ],
    },
)
