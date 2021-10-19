from os.path import join
from setuptools import setup

package_name = "gordon_control"

setup(
    name=package_name,
    version="1.0.0",
    data_files=[
        (join("share", package_name), ["package.xml"]),
        ("share/ament_index/resource_index/packages", [join("resource", package_name)]),
    ],
    description=package_name,
    entry_points={
        "console_scripts": [
            "drive_controller = gordon_control.nodes.drive_controller_node:main",
            "wheel_controller = gordon_control.nodes.wheel_controller_node:main",
            "ultrasonic_controller = gordon_control.nodes.ultrasonic_controller_node:main",
            "gate_controller = gordon_control.nodes.gate_controller_node:main",
        ]
    },
    install_requires=["setuptools"],
    license="MIT",
    maintainer="Scott Moore",
    maintainer_email="smoo38@student.monash.edu",
    packages=[package_name],
    zip_safe=True,
)
