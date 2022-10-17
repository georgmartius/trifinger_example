import os
import setuptools

PACKAGE_NAME = "trifinger_example"

setuptools.setup(
    name=PACKAGE_NAME,
    version="1.0.0",
    # Packages to export
    packages=setuptools.find_packages(),
    data_files=[
        # Install "marker" file in package index
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + PACKAGE_NAME],
        ),
        # Include our package.xml file
        (os.path.join("share", PACKAGE_NAME), ["package.xml"]),
    ],
    # This is important as well
    install_requires=["setuptools", "trifinger_simulation", "numpy-quaternion", "quaternion"],
    zip_safe=True,
    description="Example package for TriFinger robot.",
    license="BSD 3-clause",
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        "console_scripts": [
            "run_robot = trifinger_example.scripts.run_robot:main"
        ],
    },
)
