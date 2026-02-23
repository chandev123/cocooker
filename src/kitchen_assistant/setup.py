from setuptools import setup
import os
from glob import glob

package_name = "kitchen_assistant"

setup(
    name=package_name,
    version="0.1.0",
    packages=[
        package_name,
        package_name + ".legacy",
        package_name + ".nodes",
        package_name + ".voice",
        package_name + ".tools",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "resource"), glob("resource/*")),
        # install models
        (os.path.join("share", package_name, "models"), glob("kitchen_assistant/models/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Kitchen assistant (legacy scripts wrapped as ROS2 ament_python package).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "kitchen_cli = kitchen_assistant.nodes.kitchen_cli:main",
            "kitchen_voice = kitchen_assistant.nodes.kitchen_voice:main",
        ],
    },
)
