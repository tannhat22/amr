import os
from glob import glob
from setuptools import find_packages, setup

package_name = "amr_fleet_adapter"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["config.yaml"]),
        ("share/" + package_name, ["charge_schedule.yaml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.xml")),
    ],
    install_requires=["setuptools", "fastapi>=0.79.0", "uvicorn>=0.18.2"],
    zip_safe=True,
    maintainer="tannhat",
    maintainer_email="nguyentannhat2298@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fleet_adapter=amr_fleet_adapter.fleet_adapter:main",
            "fleet_manager=amr_fleet_adapter.fleet_manager:main",
            "fleet_conflicts=amr_fleet_adapter.fleet_conflicts_handle:main",
            "test_action_execution_notice=amr_fleet_adapter.test_action_execution_notice:main",
        ],
    },
)
