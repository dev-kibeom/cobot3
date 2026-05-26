from setuptools import setup
import os
from glob import glob

package_name = "bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=[],  # bringup은 보통 파이썬 노드 코드가 없으므로 비워두거나 [package_name]으로 둡니다.
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # 빌드 시 런치와 설정을 share 폴더로 복사합니다.
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
        (os.path.join("share", package_name, "rviz2"), glob("rviz2/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kibeom",
    description="Bringup package for Smart Factory",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # bringup에는 파이썬 실행 파일이 없으므로 비워둡니다.
        ],
    },
)
