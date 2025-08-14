import os
from glob import glob
from setuptools import find_packages, setup

package_name = "amr_charging_manager"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["config.yaml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.xml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tannhat",
    maintainer_email="nguyentannhat2298@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "charging_manager=amr_charging_manager.charging_manager:main",
            "fleet_data_test=amr_charging_manager.fleet_data_test:main",
        ],
    },
)
