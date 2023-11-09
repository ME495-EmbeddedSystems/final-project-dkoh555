from setuptools import find_packages, setup

package_name = "write_letters"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "launch/writer.launch.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jliu",
    maintainer_email="JingkunLiu2025@u.northwestern.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["writer = write_letters.write:main"],
    },
)
