from setuptools import find_packages, setup

package_name = "dji_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="edr",
    maintainer_email="65714311+edouardrolland@users.noreply.github.com",
    description="WildBridge ROS 2 node driving one DJI drone through the WildBridge HTTP command and telemetry interface",
    license="MIT",
    entry_points={
        "console_scripts": ["dji_node = dji_controller.controller:main"],
    },
)
