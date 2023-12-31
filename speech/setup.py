from setuptools import find_packages, setup

package_name = 'speech'

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
    maintainer='Henry Buron',
    maintainer_email='henryburon2024@u.northwestern.edu',
    description='Handles speech function of polyglotbot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listen_speech = speech.listen_speech:listen_speech_entry'
        ],
    },
)
