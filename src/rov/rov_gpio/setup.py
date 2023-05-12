from setuptools import setup

package_name = 'rov_gpio'

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
    description='GPIO Interface for the ROV',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rov_gpio = rov_gpio.RovGpio:main'
        ],
    },
)
