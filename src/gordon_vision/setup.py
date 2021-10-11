from os.path import join
from setuptools import setup

package_name = "gordon_vision"

setup(
    name=package_name,
    version="1.0.0",
    data_files=[
        (join("share", package_name), ["package.xml"]),
        (
            "share/ament_index/resource_index/packages",
            [join("resource", package_name)],
        ),
    ],
    description=package_name,
    entry_points={"console_scripts": ["vision = gordon_vision.vision:main"]},
    install_requires=["setuptools"],
    license="MIT",
    maintainer="Scott Moore",
    maintainer_email="smoo38@student.monash.edu",
    packages=[package_name],
    tests_require=["pytest"],
    zip_safe=True,
)
