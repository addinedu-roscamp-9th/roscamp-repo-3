from setuptools import find_packages, setup

PKG_NAME = "debugcrew"

setup(
    name=PKG_NAME,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PKG_NAME]),
        ("share/" + PKG_NAME, ["package.xml"]),
        ("share/" + PKG_NAME + "/launch", ["launch/pinky.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="KimChanHee",
    maintainer_email="blackeagle10@icloud.com",
    description="TODO: Package description",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "vel_sub = debugcrew.vel_sub:main",
            "pid_node = debugcrew.pid_node:main",
        ],
    },
)
