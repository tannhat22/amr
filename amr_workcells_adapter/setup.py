import os
from glob import glob
from setuptools import find_packages, setup

package_name = "amr_workcells_adapter"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tannhat",
    maintainer_email="nguyentannhat2298@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dispenser_adapter=amr_workcells_adapter.dispenser_adapter:main",
            "ingestor_adapter=amr_workcells_adapter.ingestor_adapter:main",
            "workcells_adapter=amr_workcells_adapter.workcells_adapter:main",
        ],
    },
)
