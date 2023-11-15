from setuptools import find_packages, setup
import glob
import os

package_name = 'polyglotbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*launch.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='megan',
    maintainer_email='meganblack2027@u.northwestern.edu',
    description='Combines individual package functions to read, translate, and write text',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'polyglotbot = polyglotbot.polyglotbot:entry_point'
        ],
    },
)
