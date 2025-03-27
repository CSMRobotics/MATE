from setuptools import find_packages, setup

package_name = 'led_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # add WS2812
        ('lib/' + package_name, [package_name+'/WS2812.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='catfishjw',
    maintainer_email='james.wiley@live.com',
    description='It controls LEDs',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_controller = led_controller.led_controller:main'
        ],
    },
)
