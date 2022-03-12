from setuptools import setup

package_name = 'rov_estop'

setup(
    name=package_name,
    version='1.0.0',
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
    description='Emergency Stop for the ROV side of things',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rov_estop = rov_estop.rov_estop:main'
        ],
    },
)
