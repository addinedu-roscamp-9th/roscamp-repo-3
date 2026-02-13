from setuptools import setup

package_name = "pinky_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pinky",
    maintainer_email="pinky@todo.todo",
    description="Pinky manual control logger",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pinky_logger = pinky_control.pinky_logger:main",
            "pinky_status_sender = pinky_control.pinky_status_sender:main",
            "pinky_cmd_listener = pinky_control.cmd_listener:main",
            "goto_pose = pinky_control.goto_pose:main",
            "nav_goal_listener = pinky_control.nav_goal_listener:main",
        ],
    },
)
