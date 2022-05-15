import os
from setuptools import setup
from glob import glob

package_name = 'bs_flask'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/static/', glob("static/*")),
        ('share/' + package_name, ["default.jpg","stylesheet.css","frozen.jpg"])

    ].__add__([('share/' + package_name + '/' + directory, [os.path.join(directory, file) for file in files]) for directory, _, files in os.walk("templates")]),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yameat',
    maintainer_email='zac71113@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bs_flask = bs_flask.bs_flask:main'
        ],
    },
)
