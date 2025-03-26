from setuptools import find_packages, setup

package_name = 'rov_tsys01'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='catfishjw',
    maintainer_email='james.wiley@live.com',
    description='library and node for TSYS01 Temperature Sensor',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tsys01 = rov_tsys01.main:main'
        ],
    },
)
