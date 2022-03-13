from setuptools import setup

package_name = 'bar02'

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
    maintainer='Zac Stanton',
    maintainer_email='zac71113@gmail.com',
    description='ROS 2 package for MS5837-02BA Blue Robotics Depth Sensor',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bar02 = bar02.bar02:main'
        ],
    },
)
