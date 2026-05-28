from setuptools import find_packages, setup

package_name = 'bridge'

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
    description="데이터 전처리 및 통신",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "mqtt_trigger = bridge.mqtt_trigger:main",
            "mqtt_receiver = bridge.mqtt_receiver:main",
            "image_bridge = bridge.image_bridge_node:main",
        ],
    },
)
