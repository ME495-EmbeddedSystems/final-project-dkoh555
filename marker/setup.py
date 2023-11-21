from setuptools import find_packages, setup

package_name = 'marker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'launch/marker.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='megan',
    maintainer_email='meganblack2027@u.northwestern.edu',
    description='Picks up a marker',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker = marker.marker:entry_point'
        ],
    },
)
