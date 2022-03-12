from setuptools import setup

package_name = 'flight_controller'

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
    description="Controller for robot 'flight', or 'swimming', or whatever word you prefer. Takes in orientation information and translate/rotate commands to converts them to create smooth, controlled motion through the water.  Or at least, that's the idea",
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flightcontroller = flight_controller.flightcontroller:main'
        ],
    },
)
