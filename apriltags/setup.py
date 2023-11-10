from setuptools import find_packages, setup

package_name = 'apriltags'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'launch/get_apriltags.launch.xml',
                                   'rviz/view_apriltags.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Henry Buron',
    maintainer_email='henryburon2024@u.northwestern.edu',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_apriltags = apriltags.get_apriltags:get_apriltags_entry'
        ],
    },
)
