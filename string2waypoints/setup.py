from setuptools import find_packages, setup

package_name = 'string2waypoints'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'launch/create_waypoint.launch.xml',
                                   'Fonts/Roboto-Thin.ttf',
                                   'Fonts/NotoSansTC-VariableFont_wght.ttf',
                                   'Fonts/NotoSansSC-VariableFont_wght.ttf',
                                   'Fonts/NotoSansKR-VariableFont_wght.ttf',
                                   'Fonts/NotoSansJP-VariableFont_wght.ttf',
                                   'Fonts/NotoSansHebrew-VariableFont_wdth,wght.ttf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kassidy Shedd',
    maintainer_email='kass92800@gmail.com',
    description='This node takes a string as an input and will output corresponding waypoints allowing the robot to write the string.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'create_waypoint = string2waypoints.string2waypoint_node:main'
        ],
    },
)
