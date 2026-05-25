from setuptools import find_packages, setup

package_name = 'controller'

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kibeom",
    maintainer_email="neopkrrl@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "pick_and_place_joint_node = controller.pick_and_place_joint:main",
            "pick_and_place_pos_node = controller.pick_and_place_pos:main",
        ],
    },
)
